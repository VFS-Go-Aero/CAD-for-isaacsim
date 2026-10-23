#!/usr/bin/env python3
"""
Battery + Jetson companion-computer power model for the HIL bridge.

Answers a power-budget question: with this battery, harness and regulator,
does the Jetson stay powered through a flight, including gradual battery sag,
scripted brownouts and random voltage transients?

  battery: OCV(SoC), R_cells
    └─ harness R ── main bus ─┬─ 4x ESC + motor  (power from PX4 thrust)
                              ├─ other avionics  (constant W)
                              └─ [injected drop] ─ buck regulator ─ hold-up cap ─ Jetson

The model only observes the flight: it reads per-motor thrust but never feeds
voltage back into thrust or PX4, so enabling it does not change how the
vehicle flies. It has no omni/numpy imports, so px4_bridge.py can validate the
config before Isaac Sim boots, and it also runs on its own for quick sizing
sweeps at constant hover thrust:

  python power_model.py --config power_config.example.json --duration 600

Every default below is a placeholder. Replace them with the real battery,
regulator and Jetson datasheet values before trusting a verdict.
"""
import argparse
import bisect
import csv
import json
import math
import random
import sys
from dataclasses import dataclass, field, fields

# ============================================================
# Constants
# ============================================================
# Generic LiPo open-circuit voltage per cell vs state of charge. Override with
# battery.ocv_table using your cell's datasheet curve (Li-ion packs sit
# noticeably lower near empty).
LIPO_OCV = (
    (0.00, 3.30), (0.05, 3.60), (0.10, 3.69), (0.20, 3.74), (0.30, 3.77),
    (0.40, 3.80), (0.50, 3.84), (0.60, 3.87), (0.70, 3.92), (0.80, 3.98),
    (0.90, 4.06), (1.00, 4.20),
)

MAX_STEP_FAULTS = 1000
MAX_TRANSIENT_RATE_HZ = 1000.0

JETSON_ON = 'ON'
JETSON_OFF = 'OFF'
JETSON_BOOTING = 'BOOTING'

LOG_COLUMNS = (
    't_s', 'armed', 'soc', 'ocv_v', 'bus_v', 'current_a', 'motor_w', 'jetson_w',
    'injected_drop_v', 'reg_in_v', 'reg_out_v', 'jetson_in_v', 'jetson_state',
)


# ============================================================
# Config
# ============================================================
# Float fields must keep a decimal point in their default: _load_section uses
# the default's type to decide how to validate the JSON value.
@dataclass
class BatteryConfig:
    cells: int = 6                  # series cell count (6S)
    capacity_ah: float = 10.0
    cell_r_ohm: float = 0.004       # DC internal resistance per cell
    harness_r_ohm: float = 0.010    # main leads + connectors, battery to bus
    initial_soc: float = 1.0        # start lower to test end-of-flight sag
    drain_speedup: float = 1.0      # >1 squeezes a full discharge into a short sim
    ocv_table: list = field(default_factory=lambda: [list(p) for p in LIPO_OCV])

    def validate(self, name):
        _require(1 <= self.cells <= 30, f'{name}.cells must be 1..30')
        _require(self.capacity_ah > 0, f'{name}.capacity_ah must be > 0')
        _require(self.cell_r_ohm >= 0, f'{name}.cell_r_ohm must be >= 0')
        _require(self.harness_r_ohm >= 0, f'{name}.harness_r_ohm must be >= 0')
        _require(0 <= self.initial_soc <= 1, f'{name}.initial_soc must be 0..1')
        _require(self.drain_speedup > 0, f'{name}.drain_speedup must be > 0')
        table = []
        for i, point in enumerate(self.ocv_table):
            where = f'{name}.ocv_table[{i}]'
            _require(isinstance(point, list) and len(point) == 2,
                     f'{where} must be a [soc, cell_v] pair')
            table.append((_as_number(point[0], where), _as_number(point[1], where)))
        _require(len(table) >= 2, f'{name}.ocv_table needs at least 2 points')
        _require(table[0][0] == 0.0 and table[-1][0] == 1.0,
                 f'{name}.ocv_table must start at soc 0 and end at soc 1')
        for (s0, v0), (s1, v1) in zip(table, table[1:]):
            _require(s1 > s0, f'{name}.ocv_table soc values must strictly increase')
            _require(v1 >= v0, f'{name}.ocv_table voltages must not decrease with soc')
        _require(table[0][1] > 0, f'{name}.ocv_table voltages must be > 0')
        self.ocv_table = table


@dataclass
class PropulsionConfig:
    prop_diameter_m: float = 0.81   # README "Prop span"
    figure_of_merit: float = 0.70   # rotor efficiency vs ideal momentum theory
    drive_efficiency: float = 0.80  # motor x ESC
    air_density: float = 1.225      # kg/m^3, sea level ISA

    def validate(self, name):
        _require(self.prop_diameter_m > 0, f'{name}.prop_diameter_m must be > 0')
        _require(0 < self.figure_of_merit <= 1, f'{name}.figure_of_merit must be 0..1')
        _require(0 < self.drive_efficiency <= 1, f'{name}.drive_efficiency must be 0..1')
        _require(self.air_density > 0, f'{name}.air_density must be > 0')


@dataclass
class AvionicsConfig:
    power_w: float = 10.0           # flight controller, GPS, radios, sensors

    def validate(self, name):
        _require(self.power_w >= 0, f'{name}.power_w must be >= 0')


@dataclass
class RegulatorConfig:
    output_v: float = 12.0          # buck output feeding the Jetson carrier
    dropout_v: float = 1.0          # needs input >= output_v + dropout_v to regulate
    efficiency: float = 0.90

    def validate(self, name):
        _require(self.output_v > 0, f'{name}.output_v must be > 0')
        _require(self.dropout_v >= 0, f'{name}.dropout_v must be >= 0')
        _require(0 < self.efficiency <= 1, f'{name}.efficiency must be 0..1')


@dataclass
class JetsonConfig:
    power_w: float = 25.0           # module + carrier + peripherals at your power mode
    min_input_v: float = 9.0        # carrier board's minimum input voltage
    holdup_uf: float = 1000.0       # bulk capacitance at the Jetson input
    boot_time_s: float = 30.0       # power-on to software ready
    restart_margin_v: float = 0.5   # supply must recover this far above min to reboot

    def validate(self, name):
        _require(self.power_w >= 0, f'{name}.power_w must be >= 0')
        _require(self.min_input_v > 0, f'{name}.min_input_v must be > 0')
        _require(self.holdup_uf >= 0, f'{name}.holdup_uf must be >= 0')
        _require(self.boot_time_s >= 0, f'{name}.boot_time_s must be >= 0')
        _require(self.restart_margin_v >= 0, f'{name}.restart_margin_v must be >= 0')


@dataclass
class StepFault:
    t_start_s: float = 0.0          # seconds after PX4 first arms
    duration_s: float = 0.5
    drop_v: float = 0.0             # volts removed from the regulator input

    def validate(self, name):
        _require(self.t_start_s >= 0, f'{name}.t_start_s must be >= 0')
        _require(self.duration_s > 0, f'{name}.duration_s must be > 0')
        _require(self.drop_v >= 0, f'{name}.drop_v must be >= 0')


@dataclass
class TransientConfig:
    rate_hz: float = 0.0            # mean events per second (Poisson); 0 = off
    drop_v_min: float = 1.0
    drop_v_max: float = 5.0
    duration_ms_min: float = 1.0
    duration_ms_max: float = 20.0
    seed: int = 0                   # same seed -> same event sequence

    def validate(self, name):
        _require(0 <= self.rate_hz <= MAX_TRANSIENT_RATE_HZ,
                 f'{name}.rate_hz must be 0..{MAX_TRANSIENT_RATE_HZ:g}')
        _require(0 <= self.drop_v_min <= self.drop_v_max,
                 f'{name} needs 0 <= drop_v_min <= drop_v_max')
        _require(0 < self.duration_ms_min <= self.duration_ms_max,
                 f'{name} needs 0 < duration_ms_min <= duration_ms_max')


@dataclass
class FaultConfig:
    steps: list = field(default_factory=list)
    transients: TransientConfig = field(default_factory=TransientConfig)


@dataclass
class PowerConfig:
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    propulsion: PropulsionConfig = field(default_factory=PropulsionConfig)
    avionics: AvionicsConfig = field(default_factory=AvionicsConfig)
    regulator: RegulatorConfig = field(default_factory=RegulatorConfig)
    jetson: JetsonConfig = field(default_factory=JetsonConfig)
    faults: FaultConfig = field(default_factory=FaultConfig)


SECTIONS = {
    'battery': BatteryConfig,
    'propulsion': PropulsionConfig,
    'avionics': AvionicsConfig,
    'regulator': RegulatorConfig,
    'jetson': JetsonConfig,
}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _as_number(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{where} must be a number')
    try:
        value = float(value)
    except OverflowError:
        raise ValueError(f'{where} is out of range') from None
    if not math.isfinite(value):
        # json parses 1e400 as inf even with NaN/Infinity literals rejected.
        raise ValueError(f'{where} must be finite')
    return value


def _strip_comments(raw):
    """Drop "_comment"-style keys so example configs can annotate themselves."""
    return {k: v for k, v in raw.items() if not k.startswith('_')}


def _load_section(cls, raw, name):
    if not isinstance(raw, dict):
        raise ValueError(f'{name} must be a JSON object')
    raw = _strip_comments(raw)
    unknown = sorted(set(raw) - {f.name for f in fields(cls)})
    if unknown:
        raise ValueError(f'{name}: unknown key(s) {", ".join(unknown)}')
    obj = cls()
    for key, value in raw.items():
        where = f'{name}.{key}'
        default = getattr(obj, key)
        if isinstance(default, float):
            value = _as_number(value, where)
        elif isinstance(default, int):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{where} must be an integer')
        elif isinstance(default, list) and not isinstance(value, list):
            raise ValueError(f'{where} must be a list')
        setattr(obj, key, value)
    obj.validate(name)
    return obj


def _load_faults(raw, name):
    if not isinstance(raw, dict):
        raise ValueError(f'{name} must be a JSON object')
    raw = _strip_comments(raw)
    unknown = sorted(set(raw) - {'steps', 'transients'})
    if unknown:
        raise ValueError(f'{name}: unknown key(s) {", ".join(unknown)}')
    steps_raw = raw.get('steps', [])
    if not isinstance(steps_raw, list) or len(steps_raw) > MAX_STEP_FAULTS:
        raise ValueError(f'{name}.steps must be a list of at most {MAX_STEP_FAULTS} items')
    steps = [_load_section(StepFault, s, f'{name}.steps[{i}]') for i, s in enumerate(steps_raw)]
    return FaultConfig(
        steps=sorted(steps, key=lambda s: s.t_start_s),
        transients=_load_section(TransientConfig, raw.get('transients', {}),
                                 f'{name}.transients'),
    )


def _reject_constant(name):
    raise ValueError(f'{name} is not allowed in the power config')


def load_config(path=None):
    """Load and validate a power config. None -> all defaults.

    Plain JSON only (no code execution); unknown keys, wrong types and
    out-of-range values raise ValueError so a typo can't silently fall back
    to a default.
    """
    raw = {}
    if path:
        with open(path, encoding='utf-8') as f:
            raw = json.load(f, parse_constant=_reject_constant)
        if not isinstance(raw, dict):
            raise ValueError('top level must be a JSON object')
    raw = _strip_comments(raw)
    unknown = sorted(set(raw) - set(SECTIONS) - {'faults'})
    if unknown:
        raise ValueError(f'unknown section(s) {", ".join(unknown)}')

    cfg = PowerConfig(
        **{name: _load_section(cls, raw.get(name, {}), name) for name, cls in SECTIONS.items()},
        faults=_load_faults(raw.get('faults', {}), 'faults'),
    )
    _require(cfg.jetson.min_input_v + cfg.jetson.restart_margin_v <= cfg.regulator.output_v,
             'jetson.min_input_v + jetson.restart_margin_v must not exceed '
             'regulator.output_v, or the Jetson could never power on')
    return cfg


# ============================================================
# Model
# ============================================================
class PowerModel:
    """Steps the battery -> bus -> regulator -> Jetson chain once per physics tick."""

    def __init__(self, cfg, log_path=None):
        self.cfg = cfg
        b = cfg.battery
        p = cfg.propulsion
        self.r_series = b.cells * b.cell_r_ohm + b.harness_r_ohm
        disk_area = math.pi * (p.prop_diameter_m / 2.0) ** 2
        # Momentum theory: P_ideal = T^1.5 / sqrt(2 rho A) per rotor.
        self._motor_k = 1.0 / (math.sqrt(2.0 * p.air_density * disk_area)
                               * p.figure_of_merit * p.drive_efficiency)
        self._ocv_soc = [s for s, _ in b.ocv_table]
        self._ocv_v = [v for _, v in b.ocv_table]
        self._cap_f = cfg.jetson.holdup_uf * 1e-6

        self.t = 0.0
        self.soc = b.initial_soc
        self.jetson_state = JETSON_ON   # assume it booted before the flight
        self.jetson_in_v = cfg.regulator.output_v
        self._boot_elapsed = 0.0
        self.bus_v = self._cell_ocv(self.soc) * b.cells

        # Fault injection. The fault clock starts when PX4 first arms so step
        # times line up with the flight, not with how long the bridge idled.
        self.fault_t0 = None
        self._next_step = 0
        self._next_transient = None
        self._active = []               # (end_time, drop_v)
        self._rng = random.Random(cfg.faults.transients.seed)

        # Stats for the summary
        self.ah_drawn = 0.0
        self.wh_drawn = 0.0
        self.peak_a = 0.0
        self.peak_w = 0.0
        self.min_bus_v = math.inf
        self.min_bus_t = 0.0
        self.min_reg_in_v = math.inf
        self.min_headroom_v = math.inf  # jetson_in_v - min_input_v while powered
        self.brownouts = 0
        self.first_brownout_t = None
        self.downtime_s = 0.0
        self.collapse_s = 0.0
        self.step_faults_fired = 0
        self.transients_fired = 0

        self._closed = False
        self._log_file = None
        self._log = None
        if log_path:
            self._log_file = open(log_path, 'w', newline='', encoding='utf-8')
            self._log = csv.writer(self._log_file)
            self._log.writerow(LOG_COLUMNS)

    def _cell_ocv(self, soc):
        soc = min(1.0, max(0.0, soc))
        i = min(bisect.bisect_right(self._ocv_soc, soc), len(self._ocv_soc) - 1)
        s0, s1 = self._ocv_soc[i - 1], self._ocv_soc[i]
        v0, v1 = self._ocv_v[i - 1], self._ocv_v[i]
        return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)

    def motor_power_w(self, thrust_n):
        return max(0.0, thrust_n) ** 1.5 * self._motor_k

    def _fault_drop(self, dt):
        """Largest active injected drop. Events last at least one step, so
        anything shorter than dt is rounded up (conservative for sizing)."""
        if self.fault_t0 is None:
            return 0.0
        ft = self.t - self.fault_t0
        steps = self.cfg.faults.steps
        while self._next_step < len(steps) and steps[self._next_step].t_start_s <= ft:
            s = steps[self._next_step]
            self._active.append((self.t + max(s.duration_s, dt), s.drop_v))
            self._next_step += 1
            self.step_faults_fired += 1

        tr = self.cfg.faults.transients
        while self._next_transient is not None and self._next_transient <= ft:
            duration = self._rng.uniform(tr.duration_ms_min, tr.duration_ms_max) / 1000.0
            drop = self._rng.uniform(tr.drop_v_min, tr.drop_v_max)
            self._active.append((self.t + max(duration, dt), drop))
            self.transients_fired += 1
            self._next_transient += self._rng.expovariate(tr.rate_hz)

        self._active = [(end, d) for end, d in self._active if end > self.t]
        return max((d for _, d in self._active), default=0.0)

    def step(self, dt, thrusts_n, armed=True):
        """Advance by dt seconds given per-motor thrust (N) and PX4 arm state."""
        cfg = self.cfg
        b, reg, jet = cfg.battery, cfg.regulator, cfg.jetson

        if armed and self.fault_t0 is None:
            self.fault_t0 = self.t
            if cfg.faults.transients.rate_hz > 0:
                self._next_transient = self._rng.expovariate(cfg.faults.transients.rate_hz)

        # Loads on the main bus. All are constant-power, so current rises as
        # the bus sags; the Jetson's draw uses last step's on/off state.
        motor_w = sum(self.motor_power_w(t) for t in thrusts_n)
        jetson_w = jet.power_w if self.jetson_state != JETSON_OFF else 0.0
        load_w = motor_w + cfg.avionics.power_w + jetson_w / reg.efficiency

        # Battery terminal voltage: solve V = OCV - R * P / V for the upper root.
        ocv = self._cell_ocv(self.soc) * b.cells
        r = self.r_series
        disc = ocv * ocv - 4.0 * r * load_w
        if disc >= 0.0:
            bus_v = (ocv + math.sqrt(disc)) / 2.0
            current = load_w / bus_v
        else:
            # Past the pack's maximum power point: it cannot deliver load_w.
            bus_v = ocv / 2.0
            current = ocv / (2.0 * r)
            self.collapse_s += dt
        self.bus_v = bus_v

        self.soc = max(0.0, self.soc - current * dt / (3600.0 * b.capacity_ah) * b.drain_speedup)
        self.ah_drawn += current * dt / 3600.0
        self.wh_drawn += bus_v * current * dt / 3600.0

        # Regulator input = bus minus any injected drop; buck regulates until
        # it runs out of headroom, then its output tracks the input.
        drop_v = self._fault_drop(dt)
        reg_in_v = max(0.0, bus_v - drop_v)
        reg_out_v = max(0.0, min(reg.output_v, reg_in_v - reg.dropout_v))

        # Hold-up capacitor rides the Jetson through dips shorter than
        # C * (V^2 - Vmin^2) / (2 P).
        if reg_out_v >= self.jetson_in_v or self._cap_f == 0.0:
            self.jetson_in_v = reg_out_v
        else:
            energy = 0.5 * self._cap_f * self.jetson_in_v ** 2 - jetson_w * dt
            self.jetson_in_v = max(reg_out_v, math.sqrt(max(0.0, 2.0 * energy / self._cap_f)))

        if self.jetson_state != JETSON_OFF and self.jetson_in_v < jet.min_input_v:
            self.jetson_state = JETSON_OFF
            self.brownouts += 1
            if self.first_brownout_t is None:
                self.first_brownout_t = self.t
        elif self.jetson_state == JETSON_OFF and reg_out_v >= jet.min_input_v + jet.restart_margin_v:
            self.jetson_state = JETSON_BOOTING
            self._boot_elapsed = 0.0
            self.jetson_in_v = reg_out_v
        elif self.jetson_state == JETSON_BOOTING:
            self._boot_elapsed += dt
            if self._boot_elapsed >= jet.boot_time_s:
                self.jetson_state = JETSON_ON

        if self.jetson_state != JETSON_ON:
            self.downtime_s += dt
        if self.jetson_state != JETSON_OFF:
            self.min_headroom_v = min(self.min_headroom_v, self.jetson_in_v - jet.min_input_v)
        if bus_v < self.min_bus_v:
            self.min_bus_v, self.min_bus_t = bus_v, self.t
        self.min_reg_in_v = min(self.min_reg_in_v, reg_in_v)
        self.peak_a = max(self.peak_a, current)
        self.peak_w = max(self.peak_w, load_w)

        if self._log is not None:
            self._log.writerow((
                f'{self.t:.4f}', int(armed), f'{self.soc:.4f}', f'{ocv:.3f}', f'{bus_v:.3f}',
                f'{current:.2f}', f'{motor_w:.1f}', f'{jetson_w:.1f}', f'{drop_v:.3f}',
                f'{reg_in_v:.3f}', f'{reg_out_v:.3f}', f'{self.jetson_in_v:.3f}',
                self.jetson_state,
            ))
        self.t += dt

    def status(self):
        return f'bus={self.bus_v:5.2f}V soc={self.soc * 100:3.0f}% jetson={self.jetson_state}'

    def holdup_ms(self):
        jet = self.cfg.jetson
        if jet.power_w == 0.0:
            return math.inf
        v0 = self.cfg.regulator.output_v
        return 1000.0 * self._cap_f * (v0 ** 2 - jet.min_input_v ** 2) / (2.0 * jet.power_w)

    def passed(self):
        return self.brownouts == 0 and self.collapse_s == 0.0

    def summary_lines(self):
        b, jet = self.cfg.battery, self.cfg.jetson
        armed_at = 'never armed' if self.fault_t0 is None else f'faults armed at t={self.fault_t0:.1f}s'
        lines = [
            '---- power budget summary ----',
            f'sim time         {self.t:.1f} s ({armed_at})',
            f'battery          {b.cells}S {b.capacity_ah:g} Ah, SoC {b.initial_soc * 100:.0f}% '
            f'-> {self.soc * 100:.0f}% (drain x{b.drain_speedup:g})',
            f'energy drawn     {self.wh_drawn:.1f} Wh ({self.ah_drawn:.2f} Ah), '
            f'peak {self.peak_a:.1f} A / {self.peak_w:.0f} W',
            f'bus voltage      min {self.min_bus_v:.2f} V at t={self.min_bus_t:.1f}s',
            f'regulator input  min {self.min_reg_in_v:.2f} V (including injected drops)',
            f'Jetson headroom  min {self.min_headroom_v:+.2f} V vs its {jet.min_input_v:g} V cutoff',
            f'hold-up time     {self.holdup_ms():.2f} ms at {jet.power_w:g} W ({jet.holdup_uf:g} uF)',
            f'injected events  {self.step_faults_fired} step(s), {self.transients_fired} transient(s)',
            f'Jetson brownouts {self.brownouts}'
            + (f' (first at t={self.first_brownout_t:.1f}s)' if self.brownouts else '')
            + f', downtime {self.downtime_s:.1f} s',
        ]
        if self.collapse_s > 0.0:
            lines.append(f'VOLTAGE COLLAPSE for {self.collapse_s:.2f} s: the pack + harness '
                         f'cannot supply the load (undersized battery or wiring)')
        if self.soc == 0.0:
            lines.append('battery reached 0% SoC')
        lines.append('verdict          ' + (
            'PASS: Jetson stayed powered' if self.passed() else 'FAIL: see brownouts / collapse above'))
        return lines

    def close(self):
        """Close the CSV log and print the summary. Safe to call twice."""
        if self._closed:
            return
        self._closed = True
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
            self._log = None
        for line in self.summary_lines():
            print(f'[POWER] {line}', flush=True)


# ============================================================
# Standalone run (no Isaac Sim / PX4)
# ============================================================
def _main(argv=None):
    ap = argparse.ArgumentParser(
        description='Run the power model at constant hover thrust, without Isaac Sim. '
                    'Exits 1 if the Jetson browns out or the pack collapses.')
    ap.add_argument('--config', help='JSON power config (default: built-in placeholders)')
    ap.add_argument('--duration', type=float, default=600.0, help='Seconds to simulate (default 600)')
    ap.add_argument('--thrust-n', type=float, default=5.0 * 9.81,
                    help='Total thrust in N, split over 4 motors '
                         '(default 49.05 = 5 kg hover, matching px4_bridge.py)')
    ap.add_argument('--dt', type=float, default=0.004,
                    help='Step in seconds (default 0.004 = bridge rate; smaller resolves '
                         'shorter transients)')
    ap.add_argument('--log', help='Write a per-step CSV log here')
    args = ap.parse_args(argv)
    if not (math.isfinite(args.duration) and args.duration > 0):
        ap.error('--duration must be > 0')
    if not (math.isfinite(args.dt) and args.dt > 0):
        ap.error('--dt must be > 0')
    if not (math.isfinite(args.thrust_n) and args.thrust_n >= 0):
        ap.error('--thrust-n must be >= 0')

    try:
        cfg = load_config(args.config)
    except (OSError, ValueError) as e:
        ap.error(f'--config: {e}')
    model = PowerModel(cfg, log_path=args.log)
    per_motor = [args.thrust_n / 4.0] * 4
    for _ in range(int(round(args.duration / args.dt))):
        model.step(args.dt, per_motor, armed=True)
    model.close()
    return 0 if model.passed() else 1


if __name__ == '__main__':
    sys.exit(_main())
