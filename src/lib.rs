#![no_std]

extern crate ev3rt;
use ev3rt::*;

#[derive(PartialEq)]
pub enum ColorSensorMode {
    NONE = 0,
    COLOR,
    REFLECT,
    AMBIENT,
    RGB,
}

#[derive(PartialEq)]
pub enum IrSensorMode {
    NONE = 0,
    DISTANCE,
    SEEK,
    REMOTE,
}

#[derive(PartialEq)]
pub enum UsSensorMode {
    NONE = 0,
    DISTANCE,
}

#[derive(PartialEq)]
pub enum GyroSensorMode {
    NONE = 0,
    ANGLE,
    RATE,
}

#[derive(PartialEq)]
pub enum SensorConfiguration {
    None,
    Ultrasonic(UsSensorMode),
    Gyro(GyroSensorMode),
    Touch,
    Color(ColorSensorMode),
    Ir(IrSensorMode),
    NxtAccel,
    NxtColor,
    NxtTemp,
    NxtUltrasonic(UsSensorMode),
}

pub struct SensorData {
    port_idx: SensorPort,
    cfg: SensorConfiguration,
    cfg_applied: bool,
    pub data: [u16; 4],
}

impl SensorData {
    fn clear_data(&mut self) {
        self.data = [0xffff; 4];
    }

    pub fn new(port: SensorPort) -> Self {
        return Self {
            port_idx: port,
            cfg: SensorConfiguration::None,
            cfg_applied: false,
            data: [0xffff; 4],
        };
    }

    pub fn port(&self) -> SensorPort {
        self.port_idx
    }

    fn val_conversion(v: u16) -> i32 {
        (v as i16) as i32
    }
    pub fn val(&self) -> i32 {
        Self::val_conversion(self.data[0])
    }

    pub fn v1(&self) -> i32 {
        Self::val_conversion(self.data[0])
    }

    pub fn v2(&self) -> i32 {
        Self::val_conversion(self.data[1])
    }

    pub fn v3(&self) -> i32 {
        Self::val_conversion(self.data[2])
    }

    pub fn v4(&self) -> i32 {
        Self::val_conversion(self.data[3])
    }

    pub fn configuration_applied(&self) -> bool {
        self.cfg_applied
    }

    pub fn configure(&mut self, cfg: SensorConfiguration) {
        self.cfg = cfg;
        self.cfg_applied = false;
        self.clear_data();
    }

    pub fn attempt_cfg_apply(&mut self) -> bool {
        self.clear_data();
        self.cfg_applied = false;

        match &self.cfg {
            SensorConfiguration::None => {
                ev3rt::sensor_config(self.port(), SensorType::NONE);
                self.cfg_applied = true;
            }
            SensorConfiguration::Ultrasonic(mode) => {
                let er = ev3rt::sensor_config(self.port(), SensorType::ULTRASONIC);
                if er == ER::OK {
                    self.cfg_applied = true;
                    match mode {
                        UsSensorMode::DISTANCE => {
                            let val = ev3rt::ultrasonic_sensor_get_distance(self.port_idx);
                            if val == -1 || val == 0 {
                                self.cfg_applied = false;
                            }
                        }
                        _ => {
                            self.cfg_applied = false;
                        }
                    }
                }
            }
            SensorConfiguration::Gyro(mode) => {
                let er = ev3rt::sensor_config(self.port(), SensorType::GYRO);
                if er == ER::OK {
                    self.cfg_applied = true;
                    match mode {
                        _ => {
                            self.cfg_applied = true;
                        }
                    }
                }
            }
            SensorConfiguration::Touch => {
                let er = ev3rt::sensor_config(self.port(), SensorType::TOUCH);
                if er == ER::OK {
                    self.cfg_applied = true;
                }
                self.cfg_applied = false;
            }
            SensorConfiguration::Color(mode) => {
                let er = ev3rt::sensor_config(self.port(), SensorType::COLOR);
                if er == ER::OK {
                    self.cfg_applied = true;
                    match mode {
                        ColorSensorMode::AMBIENT => {
                            let val = ev3rt::color_sensor_get_ambient(self.port_idx);
                            if val == 0xff || val == 0 {
                                self.cfg_applied = false;
                            }
                        }
                        ColorSensorMode::COLOR => {
                            let val = ev3rt::color_sensor_get_color(self.port_idx);
                            if let ev3rt::SensorColorCode::NONE = val {
                                self.cfg_applied = false;
                            }
                        }
                        ColorSensorMode::REFLECT => {
                            let val = ev3rt::color_sensor_get_reflect(self.port_idx);
                            if val == 0xff || val == 0 {
                                self.cfg_applied = false;
                            }
                        }
                        ColorSensorMode::RGB => {
                            let val = ev3rt::color_sensor_get_rgb(self.port_idx);
                            if val.r == 0xffff
                                || val.g == 0xffff
                                || val.b == 0xffff
                                || (val.r == 0 && val.g == 0 && val.b == 0)
                            {
                                self.cfg_applied = false;
                            }
                        }
                        _ => {}
                    }
                }
            }
            SensorConfiguration::Ir(mode) => {
                let er = ev3rt::sensor_config(self.port(), SensorType::INFRARED);
                if er == ER::OK {
                    self.cfg_applied = true;
                    match mode {
                        IrSensorMode::DISTANCE => {
                            let val = ev3rt::infrared_sensor_get_distance(self.port_idx);
                            if val == 0xff || val == 0 {
                                self.cfg_applied = false;
                            }
                        }
                        IrSensorMode::REMOTE => {
                            self.cfg_applied = false;
                        }
                        IrSensorMode::SEEK => {
                            self.cfg_applied = false;
                        }
                        _ => {}
                    }
                }
            }
            SensorConfiguration::NxtAccel => {}
            SensorConfiguration::NxtColor => {}
            SensorConfiguration::NxtTemp => {}
            SensorConfiguration::NxtUltrasonic(mode) => {
                let er = ev3rt::sensor_config(self.port(), SensorType::HtNxtINFRARED);
                if er == ER::OK {
                    self.cfg_applied = true;
                    match mode {
                        UsSensorMode::DISTANCE => {
                            let val = ev3rt::htnxt_ultrasonic_sensor_get_distance(self.port_idx);
                            if val == -1 || val == 0 {
                                self.cfg_applied = false;
                            }
                        }
                        _ => {
                            self.cfg_applied = false;
                        }
                    }
                }
            }
        }
        self.cfg_applied
    }

    pub fn read(&mut self) {
        self.clear_data();
        if !self.cfg_applied {
            return;
        }

        match &self.cfg {
            SensorConfiguration::None => {}
            SensorConfiguration::Color(mode) => {
                match mode {
                    ColorSensorMode::AMBIENT => {
                        let val = ev3rt::color_sensor_get_ambient(self.port_idx);
                        if val == 0xff {
                            self.data[0] = 0xffff;
                        } else {
                            self.data[0] = val as u16;
                        }
                    }
                    ColorSensorMode::COLOR => {
                        // let val = ev3rt::color_sensor_get_color(self.port_idx);
                        // TODO: encode and decode value...
                        self.data[0] = 0xffff;
                    }
                    ColorSensorMode::REFLECT => {
                        let val = ev3rt::color_sensor_get_reflect(self.port_idx);
                        if val == 0xff {
                            self.data[0] = 0xffff;
                        } else {
                            self.data[0] = val as u16;
                        }
                    }
                    ColorSensorMode::RGB => {
                        let val = ev3rt::color_sensor_get_rgb(self.port_idx);
                        self.data[0] = val.r;
                        self.data[1] = val.g;
                        self.data[2] = val.b;
                    }
                    _ => {}
                }
            }
            SensorConfiguration::Ir(mode) => match mode {
                IrSensorMode::DISTANCE => {
                    let val = ev3rt::infrared_sensor_get_distance(self.port_idx);
                    if val == 0xff {
                        self.data[0] = 0xffff;
                    } else {
                        self.data[0] = val as u16;
                    }
                }
                IrSensorMode::REMOTE => {
                    self.clear_data();
                }
                IrSensorMode::SEEK => {
                    self.clear_data();
                }
                _ => {}
            },
            SensorConfiguration::Ultrasonic(mode) => match mode {
                UsSensorMode::DISTANCE => {
                    let val = ev3rt::ultrasonic_sensor_get_distance(self.port_idx);
                    if val == -1 {
                        self.data[0] = 0xffff;
                    } else {
                        self.data[0] = val as u16;
                    }
                }
                _ => {}
            },
            SensorConfiguration::Gyro(mode) => match mode {
                GyroSensorMode::ANGLE => {
                    self.data[0] = ev3rt::gyro_sensor_get_angle(self.port_idx) as u16;
                }
                GyroSensorMode::RATE => {
                    self.data[0] = ev3rt::gyro_sensor_get_rate(self.port_idx) as u16;
                }
                GyroSensorMode::NONE => {}
            },
            _ => {}
        }
    }
}

pub struct MotorData {
    port_idx: MotorPort,
    cfg: MotorType,
    cfg_applied: bool,
    pwr: i8,
    pos: i32,
    pos_target: i32,
    pos_i: i32,
}

impl MotorData {
    fn clear_data(&mut self) {
        self.pwr = 0;
        self.pos = 0;
    }

    pub fn new(port: MotorPort) -> MotorData {
        return MotorData {
            port_idx: port,
            cfg: MotorType::NONE,
            cfg_applied: false,
            pwr: 0,
            pos: 0,
            pos_target: 0,
            pos_i: 0,
        };
    }

    pub fn port(&self) -> MotorPort {
        self.port_idx
    }

    pub fn power(&self) -> i32 {
        self.pwr.into()
    }

    pub fn set_power(&mut self, power: i32) {
        self.pwr = if power > 100 {
            100
        } else if power < -100 {
            -100
        } else {
            power as i8
        };
    }

    pub fn apply_power(&self) {
        motor_set_power(self.port(), self.pwr.into());
    }

    pub fn set_power_to_position_fixed(&mut self, target: i32) {
        const OUTER_ZONE_POWER: i32 = 60;
        const OUTER_ZONE: i32 = 12;
        const MIDDLE_ZONE_FACTOR: i32 = 2;
        const INNER_ZONE_FACTOR: i32 = 1;
        const INNER_ZONE: i32 = 8;

        //const OUTER_ZONE_POWER: i32 = 80;
        //const OUTER_ZONE: i32 = 12;
        //const MIDDLE_ZONE_FACTOR: i32 = 3;
        //const INNER_ZONE_FACTOR: i32 = 2;
        //const INNER_ZONE: i32 = 8;

        let diff = target - self.pos;
        let pwr = if diff > OUTER_ZONE {
            OUTER_ZONE_POWER
        } else if diff > INNER_ZONE {
            diff * MIDDLE_ZONE_FACTOR
        } else if diff < -OUTER_ZONE {
            -OUTER_ZONE_POWER
        } else if diff < -INNER_ZONE {
            -diff * MIDDLE_ZONE_FACTOR
        } else {
            diff * INNER_ZONE_FACTOR
        };
        self.pwr = pwr as i8;
    }

    pub fn set_power_to_position(&mut self, target: i32, dt: Duration) {
        const KP_N: i32 = 2;
        const KP_D: i32 = 1;
        const KI_N: i32 = 1;
        const KI_D: i32 = 40000;
        const OUTER_ZONE: i32 = 10;
        const OUTER_ZONE_KP: i32 = 4;
        const OUTER_ZONE_MIN_PWR: i32 = (OUTER_ZONE * KP_N) / KP_D;
        const DIFF_MAX: i32 = 80;
        const POS_I_MAX: i32 = DIFF_MAX * 5000;
        let dt = dt.usec();

        if self.pos_target != target {
            self.pos_target = target;
            self.pos_i = 0;
        }

        let diff = target - self.pos;
        let diff = if diff > DIFF_MAX {
            DIFF_MAX
        } else if diff < -DIFF_MAX {
            -DIFF_MAX
        } else {
            diff
        };

        let pwr = if diff >= OUTER_ZONE {
            OUTER_ZONE_MIN_PWR + ((diff - OUTER_ZONE) * OUTER_ZONE_KP)
        } else if diff <= -OUTER_ZONE {
            -OUTER_ZONE_MIN_PWR + ((diff + OUTER_ZONE) * OUTER_ZONE_KP)
        } else {
            (diff * KP_N) / KP_D
        };

        self.pos_i += diff * dt;
        if self.pos_i > POS_I_MAX {
            self.pos_i = POS_I_MAX
        } else if self.pos_i < -POS_I_MAX {
            self.pos_i = -POS_I_MAX
        }

        let pwr = pwr + (self.pos_i * KI_N) / KI_D;
        self.set_power(pwr);
    }

    pub fn position(&self) -> i32 {
        self.pos
    }

    pub fn configuration_applied(&self) -> bool {
        self.cfg_applied
    }

    pub fn configure(&mut self, cfg: MotorType) {
        self.cfg = cfg;
        self.cfg_applied = false;
        self.clear_data();
    }

    pub fn attempt_cfg_apply(&mut self) {
        self.clear_data();
        let er = motor_config(self.port(), self.cfg);
        if er == ER::OK {
            self.cfg_applied = true;
            motor_stop(self.port(), false);
            match self.cfg {
                MotorType::LARGE | MotorType::MEDIUM => {
                    motor_reset_counts(self.port());
                }
                _ => {}
            }
        }
    }

    pub fn stop(&mut self, brake: bool) {
        self.pwr = 0;
        motor_stop(self.port(), brake);
    }

    pub fn read(&mut self) {
        match self.cfg {
            MotorType::LARGE | MotorType::MEDIUM => self.pos = motor_get_counts(self.port()),
            _ => {}
        }
    }

    pub fn reset_position(&mut self) {
        motor_reset_counts(self.port());
        self.pos = 0;
    }
}

#[derive(Clone, Copy, PartialEq)]
struct Point {
    x: u8,
    y: u8,
}

impl Point {
    fn new(x: i32, y: i32) -> Point {
        Point {
            x: x as u8,
            y: y as u8,
        }
    }

    fn plus(self: Point, other: Point) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    fn mov(self: Point, x: i32, y: i32) -> Point {
        Point {
            x: ((self.x as i32) + x) as u8,
            y: ((self.y as i32) + y) as u8,
        }
    }
}

const POINT_LAST: Point = Point { x: 254, y: 254 };

const POINT_NONE: Point = Point { x: 255, y: 255 };

const POINT_ZERO: Point = Point { x: 0, y: 0 };

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Gph {
    V0 = 0,
    V1 = 1,
    V2 = 2,
    V3 = 3,
    V4 = 4,
    V5 = 5,
    V6 = 6,
    V7 = 7,
    V8 = 8,
    V9 = 9,
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,
    Up,
    Right,
    Down,
    Left,
    TurnRight,
    TurnLeft,
    UpDown,
    LeftRight,
    TurnLeftRight,
    Lt,
    Gt,
    Plus,
    Minus,
    Milli,
    Wait,
    StrategyForward,
    StrategyTurnLeft,
    StrategyTurnRight,
    StrategyTurnBackLeft,
    StrategyTurnBackRight,
    Space,
    LAST,
    NONE = 0xff,
}

impl Gph {
    fn digit(value: i32) -> Gph {
        match value {
            0 => Gph::V0,
            1 => Gph::V1,
            2 => Gph::V2,
            3 => Gph::V3,
            4 => Gph::V4,
            5 => Gph::V5,
            6 => Gph::V6,
            7 => Gph::V7,
            8 => Gph::V8,
            9 => Gph::V9,
            _ => Gph::NONE,
        }
    }
}

const MAX_GLYPH_POINTS: usize = 32;
pub const GLYPH_WIDTH: i32 = 13;
pub const GLYPH_WIDTH_WIDE: i32 = 23;
pub const GLYPH_HEIGHT: i32 = 23;

#[derive(Clone, Copy)]
pub struct Glyph {
    id: Gph,
    wide: bool,
    points: [Point; MAX_GLYPH_POINTS],
}

impl Glyph {
    fn new(id: Gph, wide: bool, shape: &'static str) -> Glyph {
        let mut points = [POINT_NONE; MAX_GLYPH_POINTS];
        let mut step = 0;
        let mut x = 0;

        for (i, c) in shape.bytes().enumerate() {
            if step >= MAX_GLYPH_POINTS {
                break;
            }

            let skip = c == ('-' as u8);

            if i % 2 == 0 {
                if !skip {
                    x = (c as i32) - ('a' as i32);
                }
            } else {
                if !skip {
                    let y = (c as i32) - ('A' as i32);

                    if x >= 0
                        && x <= (MAX_GLYPH_POINTS as i32)
                        && y >= 0
                        && y <= (MAX_GLYPH_POINTS as i32)
                    {
                        points[step] = Point {
                            x: x as u8,
                            y: y as u8,
                        };
                    } else {
                        points[step] = POINT_LAST;
                    }
                } else {
                    points[step] = POINT_NONE;
                }
                step += 1;
            }
        }

        if step < (MAX_GLYPH_POINTS - 1) {
            points[step] = POINT_LAST;
        }

        Glyph { id, wide, points }
    }

    fn draw(&self, s: &Screen, x: i32, y: i32, bold: bool) {
        let origin = Point::new(x, y);

        s.clear_in_info(
            origin,
            if self.wide {
                GLYPH_WIDTH_WIDE
            } else {
                GLYPH_WIDTH
            },
            GLYPH_HEIGHT,
        );

        let base = origin.mov(1, 1);
        let mut current: Option<Point> = None;

        for p in self.points.iter() {
            if *p == POINT_LAST {
                return;
            } else if *p == POINT_NONE {
                current = None;
            } else {
                let next = base.plus(*p);
                if let Some(c) = current {
                    s.line_in_info(c, next, bold);
                }
                current = Some(next);
            }
        }
    }
}

const MAX_INFO_BOX_GLYPHS: usize = 12;
#[derive(Clone, Copy)]
pub struct InfoBox {
    size: u8,
    position: Point,

    todo_value: i32,
    done_value: i32,
    value_position: u8,
    value_digits: u8,

    has_sign: bool,
    positive_gph: Gph,
    zero_gph: Gph,
    negative_gph: Gph,

    todo_bold: bool,
    done_bold: bool,
    todo: [Gph; MAX_INFO_BOX_GLYPHS],
    done: [Gph; MAX_INFO_BOX_GLYPHS],
}

impl InfoBox {
    fn new() -> InfoBox {
        InfoBox {
            size: 0,
            position: POINT_ZERO,
            todo_value: 0,
            done_value: 0,
            value_position: 0,
            value_digits: 0,
            has_sign: false,
            positive_gph: Gph::NONE,
            zero_gph: Gph::NONE,
            negative_gph: Gph::NONE,
            todo_bold: false,
            done_bold: false,
            todo: [Gph::NONE; MAX_INFO_BOX_GLYPHS],
            done: [Gph::NONE; MAX_INFO_BOX_GLYPHS],
        }
    }

    pub fn reset(&mut self) {
        *self = InfoBox::new();
    }

    pub fn value(&self) -> i32 {
        self.todo_value
    }

    pub fn set_value(&mut self, value: i32) {
        self.todo_value = value;
    }

    pub fn setup_value(&mut self, position: u8, digits: u8) {
        self.value_position = position;
        self.value_digits = digits;
        self.has_sign = false;
        self.positive_gph = Gph::NONE;
        self.zero_gph = Gph::NONE;
        self.negative_gph = Gph::NONE;
        self.todo_value = 0;
        self.done_value = 0;
    }

    pub fn setup_signed_value(
        &mut self,
        position: u8,
        digits: u8,
        positive: Gph,
        zero: Gph,
        negative: Gph,
    ) {
        self.value_position = position + 1;
        self.value_digits = digits;
        self.has_sign = true;
        self.positive_gph = positive;
        self.zero_gph = zero;
        self.negative_gph = negative;
        self.todo_value = 0;
        self.done_value = 0;
    }

    pub fn setup_glyphs(&mut self, gphs: &[Gph]) {
        for (i, g) in gphs.iter().enumerate() {
            self.todo[i] = *g;
        }
    }

    fn apply_value(&mut self) {
        if self.value_digits <= 0 || self.todo_value == self.done_value {
            return;
        }

        let mut current_value = self.todo_value;
        if current_value < 0 {
            current_value = -current_value;
        }
        for current_digit in 0..self.value_digits {
            let current_position = self.value_position + self.value_digits - (current_digit + 1);
            let mut gph;
            gph = Gph::digit(current_value % 10);
            if gph == Gph::V0 && current_value == 0 && current_digit > 0 {
                gph = Gph::Space;
            }
            self.todo[current_position as usize] = gph;
            current_value = current_value / 10;
        }

        if self.has_sign {
            self.todo[(self.value_position - 1) as usize] = {
                if self.todo_value > 0 {
                    self.positive_gph
                } else if self.todo_value < 0 {
                    self.negative_gph
                } else {
                    self.zero_gph
                }
            };
        }

        self.done_value = self.todo_value;
    }
}

// sin(index) * 1000, with x in deg/24
const SIN_TABLE: [i32; 24] = [
    0, 258, 500, 707, 866, 965, 1000, 965, 866, 707, 500, 258, 0, -258, -500, -707, -866, -965,
    -1000, -965, -866, -707, -500, -258,
];

#[derive(Clone, Copy, PartialEq)]
pub enum ScreenOrientation {
    Up,
    Right,
    Down,
    Left,
}

const MAX_INFOS: usize = 10;
pub struct Screen {
    or: ScreenOrientation,

    must_refresh: bool,

    w: i32,
    h: i32,

    info_x: i32,
    info_y: i32,
    info_w: i32,
    info_h: i32,

    graph_x: i32,
    graph_y: i32,
    graph_ox: i32,
    graph_oy: i32,

    info_count: i32,
    infos: [InfoBox; MAX_INFOS],

    glyphs: [Glyph; Gph::LAST as usize],
}

impl Screen {
    pub fn new() -> Screen {
        let s = Screen {
            or: ScreenOrientation::Up,
            must_refresh: true,

            w: 0,
            h: 0,

            info_x: 0,
            info_y: 0,
            info_w: 0,
            info_h: 0,

            graph_x: 0,
            graph_y: 0,
            graph_ox: 0,
            graph_oy: 0,

            info_count: 0,
            infos: [InfoBox::new(); MAX_INFOS],

            //--Aa -cd-efg-hi- k
            // B-
            // CD  -cd- f -hi-
            // E-
            //  Fa -cd- f -hi- k
            // G-
            // HI  -cd- f -hi-
            // J-
            //--Ka -cd-efg-hi- k
            // L-
            // MN  -cd- f -hi-
            // O-
            //  Pa -cd- f -hi- k
            // Q-
            // RS  -cd- f -hi-
            // T-
            //--Ua -cd-efg-hi- k

            // aEcCeA -- eAgA -- gAiCkE
            // aEaG                kEkG
            // aGcIeK -- eKgK -- gKiIkG
            // aOcMeK -- eKgK -- gKiMkO
            // aQaO                kQkO
            // aQcSeU -- eUgU -- gUiSkQ
            glyphs: [
                Glyph::new(Gph::V0, false, "aDdAhAkDkRhUdUaRaD"),
                Glyph::new(Gph::V1, false, "fAfU"),
                Glyph::new(Gph::V2, false, "aDdAhAkDkHaUkU"),
                Glyph::new(Gph::V3, false, "aDdAhAkDkHhKdK--hKkNkRhUdUaR"),
                Glyph::new(Gph::V4, false, "kNaNhAhU"),
                Glyph::new(Gph::V5, false, "kAaAaKhKkNkRhUdUaR"),
                Glyph::new(Gph::V6, false, "hAdAaDaRdUhUkRkNhKdKaN"),
                Glyph::new(Gph::V7, false, "aAkAkDaU"),
                Glyph::new(Gph::V8, false, "dKaHaDdAhAkDkHhKdKaNaRdUhUkRkNhK"),
                Glyph::new(Gph::V9, false, "dUhUkRkDhAdAaDaHdKhKkH"),
                Glyph::new(Gph::A, false, "aUfAkU--dKhK"),
                Glyph::new(Gph::B, false, "aAgAiCkEkGiIgK--aUgUiSkQkOiMgK--aAaU--aKgK"),
                Glyph::new(Gph::C, false, "aEcCeAgAiCkE--aQcSeUgUiSkQ--aEaQ"),
                Glyph::new(Gph::D, false, "aAgAiCkE--aUgUiSkQ--kEkQ--aAaU"),
                Glyph::new(Gph::E, false, "aAkA--aUkU--aKkK--aAaU"),
                Glyph::new(Gph::F, false, "aAkA--aKkK--aAaU"),
                Glyph::new(Gph::G, false, "aEcCeAgAiCkE--aQcSeUgUiSkQ--aEaQ--kQkNhN"),
                Glyph::new(Gph::H, false, "aAaU--kAkU--aKkK"),
                Glyph::new(Gph::I, false, "fAfU--eAgA--eUgU"),
                Glyph::new(Gph::J, false, "aQcSeUgUiSkQkA"),
                Glyph::new(Gph::K, false, "aAaU--aKkA--aKkU"),
                Glyph::new(Gph::L, false, "aAaUkU"),
                Glyph::new(Gph::M, false, "aUaAfKkAkU"),
                Glyph::new(Gph::N, false, "aUaAkUkA"),
                Glyph::new(Gph::O, false, "aEcCeAgAiCkE--aQcSeUgUiSkQ--aEaQ--kEkQ"),
                Glyph::new(Gph::P, false, "aAgAiCkEkGiIgK--aAaU--aKgK"),
                Glyph::new(
                    Gph::Q,
                    false,
                    "aEcCeAgAiCkE--aQcSeUgUiSkQ--aEaQ--kEkQ--kUhR",
                ),
                Glyph::new(Gph::R, false, "aAgAiCkEkGiIgK--aAaU--aKgKiMkOkU"),
                Glyph::new(
                    Gph::S,
                    false,
                    "aEcCeAgAiCkE--aQcSeUgUiSkQ--aEaGcIeKgKiMkOkQ",
                ),
                Glyph::new(Gph::T, false, "aAkA--fAfU"),
                Glyph::new(Gph::U, false, "aAaQcSeUgUiSkQkA"),
                Glyph::new(Gph::V, false, "aAfUkA"),
                Glyph::new(Gph::W, false, "aAdUfKhUkA"),
                Glyph::new(Gph::X, false, "aAkU--kAaU"),
                Glyph::new(Gph::Y, false, "aAfKkA--fKfU"),
                Glyph::new(Gph::Z, false, "aAkAaUkU"),
                Glyph::new(Gph::Up, false, "fAfU--cDfAiD"),
                Glyph::new(Gph::Right, false, "aKkK--hHkKhN"),
                Glyph::new(Gph::Down, false, "fAfU--iRfUcR"),
                Glyph::new(Gph::Left, false, "aKkK--dHaKdN"),
                Glyph::new(Gph::TurnRight, false, "aNdKhKkN--gNkNkJ"),
                Glyph::new(Gph::TurnLeft, false, "aNdKhKkN--eNaNaJ"),
                Glyph::new(Gph::UpDown, false, "fAfU--cDfAiD--iRfUcR"),
                Glyph::new(Gph::LeftRight, false, "aKkK--dHaKdN--hHkKhN"),
                Glyph::new(Gph::TurnLeftRight, false, "aNdKhKkN--gNkNkJ--eNaNaJ"),
                Glyph::new(Gph::Lt, false, "kAaKkU"),
                Glyph::new(Gph::Gt, false, "aAkKaU"),
                Glyph::new(Gph::Plus, false, "aKkK--fFfP"),
                Glyph::new(Gph::Minus, false, "aKkK"),
                Glyph::new(Gph::Milli, false, "aUaK--fUfM--kUkM--aMdKfMiKkM"),
                Glyph::new(
                    Gph::Wait,
                    false,
                    "cEaCaAkAkCiE--cQaSaUkUkSiQ--cEiQ--cQiE--cEiE--cQiQ--aSkS",
                ),
                Glyph::new(Gph::StrategyForward, true, "aKkAuK--aUkKuU--kAkU"),
                Glyph::new(Gph::StrategyTurnLeft, true, "uUuRrNkKaK--kAaKkU"),
                Glyph::new(Gph::StrategyTurnRight, true, "aUaRdNkKuK--kAuKkU"),
                Glyph::new(Gph::StrategyTurnBackLeft, true, "uAuDrHkKaK--kAaKkU"),
                Glyph::new(Gph::StrategyTurnBackRight, true, "aAaDdHkKuK--kAuKkU"),
                Glyph::new(Gph::Space, false, ""),
            ],
        };
        for i in 0..(Gph::LAST as usize) {
            if s.glyphs[i].id as usize != i {
                ev3rt::lcd_clear();
                ev3rt::lcd_draw_string("BAD GLYPH IDX", 20, 20);
                ev3rt::abort();
            }
        }
        s
    }

    pub fn orientation(&self) -> ScreenOrientation {
        self.or
    }

    pub fn setup(&mut self, or: ScreenOrientation) {
        self.or = or;
        self.must_refresh = true;

        self.w = match or {
            ScreenOrientation::Up | ScreenOrientation::Down => ev3rt::LCD_WIDTH,
            ScreenOrientation::Left | ScreenOrientation::Right => ev3rt::LCD_HEIGHT,
        };
        self.h = match or {
            ScreenOrientation::Up | ScreenOrientation::Down => ev3rt::LCD_HEIGHT,
            ScreenOrientation::Left | ScreenOrientation::Right => ev3rt::LCD_WIDTH,
        };

        self.info_x = 0;
        self.info_y = 0;
        self.info_w = 0;
        self.info_h = 0;

        self.graph_x = 0;
        self.graph_y = 0;
        self.graph_ox = 0;
        self.graph_oy = 0;

        self.info_count = 0;
        self.infos = [InfoBox::new(); MAX_INFOS];
    }

    pub fn setup_info_area(&mut self, x: i32, y: i32, w: i32, h: i32) {
        self.must_refresh = true;
        self.info_x = x;
        self.info_y = y;
        self.info_w = w;
        self.info_h = h;
    }

    pub fn setup_info_area_full(&mut self) {
        self.must_refresh = true;
        self.info_x = 0;
        self.info_y = 0;
        self.info_w = self.w;
        self.info_h = self.h;
        self.graph_x = 0;
        self.graph_y = 0;
        self.graph_ox = 0;
        self.graph_oy = 0;
    }

    pub fn info_x(&self) -> i32 {
        self.info_x
    }

    pub fn info_y(&self) -> i32 {
        self.info_y
    }

    pub fn info_w(&self) -> i32 {
        self.info_w
    }

    pub fn info_h(&self) -> i32 {
        self.info_h
    }

    pub fn set_info_count(&mut self, count: i32) {
        self.must_refresh = true;
        if count < 0 {
            self.info_count = 0;
        } else if count > MAX_INFOS as i32 {
            self.info_count = MAX_INFOS as i32;
        } else {
            self.info_count = count;
        }
        self.infos = [InfoBox::new(); MAX_INFOS];
    }

    fn safe_info_index(&self, index: usize) -> usize {
        if index >= MAX_INFOS {
            MAX_INFOS - 1
        } else {
            index
        }
    }

    pub fn reset_info(&mut self, index: usize) {
        let index = self.safe_info_index(index);
        self.infos[index].reset();
    }

    pub fn setup_info(
        &mut self,
        index: usize,
        size: u8,
        row: i32,
        rows: i32,
        column: i32,
        columns: i32,
    ) {
        let index = self.safe_info_index(index);

        self.infos[index].reset();
        self.infos[index].size = size;
        let base_w = self.info_w / columns;
        let base_h = self.info_h / rows;
        let base_x = base_w * (column - 1);
        let base_y = base_h * (row - 1);
        let w = size as i32 * GLYPH_WIDTH;
        let h = GLYPH_HEIGHT;
        let x = base_x + (base_w / 2) - (w / 2);
        let y = base_y + (base_h / 2) - (h / 2);

        self.infos[index].position = Point::new(x, y);
    }

    pub fn info_value(&self, index: usize) -> i32 {
        let index = self.safe_info_index(index);
        self.infos[index].value()
    }

    pub fn set_info_value(&mut self, index: usize, value: i32) {
        let index = self.safe_info_index(index);
        self.infos[index].set_value(value);
    }

    pub fn set_info_bold(&mut self, index: usize, bold: bool) {
        let index = self.safe_info_index(index);
        self.infos[index].todo_bold = bold;
    }

    pub fn deselect_infos(&mut self) {
        for i in 0..self.info_count as usize {
            self.infos[i].todo_bold = false;
        }
    }

    pub fn select_info(&mut self, index: usize) {
        self.deselect_infos();
        let index = self.safe_info_index(index);
        self.infos[index].todo_bold = true;
    }

    pub fn setup_info_value(&mut self, index: usize, position: u8, digits: u8) {
        let index = self.safe_info_index(index);
        self.infos[index].setup_value(position, digits);
    }
    pub fn setup_info_signed_value(
        &mut self,
        index: usize,
        position: u8,
        digits: u8,
        positive: Gph,
        zero: Gph,
        negative: Gph,
    ) {
        let index = self.safe_info_index(index);
        self.infos[index].setup_signed_value(position, digits, positive, zero, negative);
    }

    pub fn setup_info_glyphs(&mut self, index: usize, gphs: &[Gph]) {
        let index = self.safe_info_index(index);
        self.infos[index].setup_glyphs(gphs);
    }

    fn render_info_box(&mut self, index: usize) {
        let index = self.safe_info_index(index);
        self.infos[index].apply_value();
        let todo_bold = self.infos[index].todo_bold;
        let done_bold = self.infos[index].done_bold;
        for i in 0..(self.infos[index].size) {
            let i = i as usize;
            let todo = self.infos[index].todo[i];
            let done = self.infos[index].done[i];

            if todo != Gph::NONE && (done != todo || done_bold != todo_bold) {
                let x = (self.infos[index].position.x as i32) + (i as i32 * GLYPH_WIDTH);
                let y = self.infos[index].position.y as i32;
                self.draw_glyph(todo, x, y, todo_bold);
                self.infos[index].done[i] = todo;
            }
        }
        self.infos[index].done_bold = todo_bold;
    }

    pub fn render_info(&mut self) {
        if self.must_refresh {
            self.clear_in_info(
                Point::new(self.info_x, self.info_y),
                self.info_w,
                self.info_h,
            );
            self.must_refresh = false;
        }
        for index in 0..self.info_count {
            self.render_info_box(index as usize);
        }
    }

    pub fn setup_graph(&mut self, x: i32, y: i32, ox: i32, oy: i32) {
        self.must_refresh = true;
        self.graph_x = x;
        self.graph_y = y;
        self.graph_ox = ox;
        self.graph_oy = oy;
    }

    pub fn info_base(&self) -> (i32, i32) {
        (self.info_x, self.info_y)
    }

    fn info_base_point(&self) -> Point {
        Point::new(self.info_x, self.info_y)
    }

    pub fn graph_base(&self) -> (i32, i32) {
        (self.graph_x, self.graph_y)
    }

    pub fn graph_origin(&self) -> (i32, i32) {
        (self.graph_x + self.graph_ox, self.graph_y + self.graph_oy)
    }

    pub fn info_origin(&self) -> (i32, i32) {
        (self.info_x, self.info_y)
    }

    // absolute coords -> real lcd screen coords
    // screens coords -> full screen coords without orientation
    // info coords -> info box screen area coords
    // graph coords -> graph box screen area coords
    fn screen_to_absolute(&self, x: i32, y: i32) -> (i32, i32) {
        match self.or {
            ScreenOrientation::Up => (x, y),
            ScreenOrientation::Down => (ev3rt::LCD_WIDTH - x, ev3rt::LCD_HEIGHT - y),
            ScreenOrientation::Left => (y, ev3rt::LCD_HEIGHT - x),
            ScreenOrientation::Right => (ev3rt::LCD_WIDTH - y, x),
        }
    }

    fn info_to_absolute_pos(&self, x: i32, y: i32) -> (i32, i32) {
        let (info_or_x, info_or_y) = self.info_origin();
        let rel_x = x + info_or_x;
        let rel_y = y + info_or_y;

        return self.screen_to_absolute(rel_x, rel_y);
    }

    fn graph_to_absolute_pos(&self, x: i32, y: i32) -> (i32, i32) {
        return match self.or {
            ScreenOrientation::Up => (x + self.graph_x, -y + self.graph_y),
            ScreenOrientation::Down => (-x + self.graph_x, y + self.graph_y),
            ScreenOrientation::Left => (y + self.graph_x, x + self.graph_y),
            ScreenOrientation::Right => (-y + self.graph_x, -x + self.graph_y),
        };
    }

    fn info_to_absolute_point(&self, p: Point) -> Point {
        let (x, y) = self.info_to_absolute_pos(p.x as i32, p.y as i32);
        Point {
            x: x as u8,
            y: y as u8,
        }
    }

    fn in_info(&self, p: Point) -> Point {
        self.info_to_absolute_point(p.plus(self.info_base_point()))
    }

    fn line_in_info(&self, p1: Point, p2: Point, bold: bool) {
        let p1 = self.in_info(p1);
        let p2 = self.in_info(p2);
        let (x0, y0, x1, y1) = (p1.x as i32, p1.y as i32, p2.x as i32, p2.y as i32);
        ev3rt::lcd_draw_line(x0, y0, x1, y1);
        if bold {
            ev3rt::lcd_draw_line(x0 + 1, y0 + 1, x1 + 1, y1 + 1);
            ev3rt::lcd_draw_line(x0 + 1, y0 - 1, x1 + 1, y1 - 1);
            ev3rt::lcd_draw_line(x0 - 1, y0 + 1, x1 - 1, y1 + 1);
            ev3rt::lcd_draw_line(x0 - 1, y0 - 1, x1 - 1, y1 - 1);
        }
    }

    pub fn box_in_info(&self, x: i32, y: i32, w: i32, h: i32, c: LcdColor) {
        let (w, h) = match self.or {
            ScreenOrientation::Up | ScreenOrientation::Down => (w, h),
            ScreenOrientation::Left | ScreenOrientation::Right => (h, w),
        };
        let (x, y) = self.info_to_absolute_pos(x, y);
        let (x, y) = match self.or {
            ScreenOrientation::Up => (x, y),
            ScreenOrientation::Down => (x - w, y - h),
            ScreenOrientation::Left => (x, y - h),
            ScreenOrientation::Right => (x - w, y),
        };
        ev3rt::lcd_fill_rect(x, y, w, h, c);
    }

    fn clear_in_info(&self, p: Point, w: i32, h: i32) {
        self.box_in_info(p.x as i32, p.y as i32, w, h, LcdColor::WHITE);
    }

    fn draw_graph_line_from_coords(&self, x0: i32, y0: i32, x1: i32, y1: i32) {
        let (x0a, y0a) = self.graph_to_absolute_pos(x0, y0);
        let (x1a, y1a) = self.graph_to_absolute_pos(x1, y1);
        ev3rt::lcd_draw_line(x0a, y0a, x1a, y1a);
    }

    pub fn draw_graph_line(&self, x0: i32, y0: i32, x1: i32, y1: i32, bold: bool) {
        self.draw_graph_line_from_coords(x0, y0, x1, y1);
        if bold {
            if y0 == y1 {
                self.draw_graph_line_from_coords(x0, y0 + 1, x1, y1 + 1);
            } else {
                self.draw_graph_line_from_coords(x0 + 1, y0, x1 + 1, y1);
            }
        }
    }

    fn draw_graph_line_from_points(&self, p0: Point, p1: Point, bold: bool) {
        self.draw_graph_line(p0.x as i32, p0.y as i32, p1.x as i32, p1.y as i32, bold)
    }

    fn move_point(from: Point, length: i32, angle: i32) -> Point {
        let mut to = from;
        to.x += (cos(angle) * length / 1000) as u8;
        to.y += (sin(angle) * length / 1000) as u8;
        return to;
    }

    pub fn draw_graph_arc(
        &self,
        center_x: i32,
        center_y: i32,
        start_angle: i32,
        arc_angle: i32,
        radius: i32,
        bold: bool,
    ) {
        let mut step = 0;
        let mut steps = 0;
        if arc_angle > 0 {
            step = 15;
            steps = (arc_angle / 15) + 1;
        } else if arc_angle < 0 {
            step = -15;
            steps = (-arc_angle / 15) + 1;
        }

        let center = Point::new(center_x, center_y);

        let mut from = Screen::move_point(center, radius, start_angle);

        for s in 0..steps {
            let to_angle = start_angle + (s * step);
            let to = Screen::move_point(center, radius, to_angle);
            self.draw_graph_line_from_points(from, to, bold);
            from = to;
        }
    }

    pub fn draw_glyph(&self, g: Gph, x: i32, y: i32, bold: bool) {
        if g as u8 >= Gph::LAST as u8 {
            return;
        }

        self.glyphs[g as usize].draw(self, x, y, bold);
    }
}

pub struct Leds {
    todo_red: bool,
    todo_green: bool,
    done_red: bool,
    done_green: bool,
}

impl Leds {
    pub fn new() -> Self {
        Leds {
            todo_red: false,
            todo_green: false,
            done_red: false,
            done_green: false,
        }
    }

    pub fn red(&self) -> bool {
        self.todo_red
    }

    pub fn green(&self) -> bool {
        self.todo_green
    }

    pub fn set_red(&mut self, v: bool) {
        self.todo_red = v;
    }

    pub fn set_green(&mut self, v: bool) {
        self.todo_green = v;
    }

    pub fn set_color(&mut self, c: LedColor) {
        let (r, g) = match c {
            LedColor::OFF => (false, false),
            LedColor::RED => (true, false),
            LedColor::GREEN => (false, true),
            LedColor::ORANGE => (true, true),
        };
        self.todo_red = r;
        self.todo_green = g;
    }

    pub fn reset(&mut self) {
        *self = Self::new();
        self.apply();
    }

    pub fn apply(&mut self) {
        if self.todo_red != self.done_red || self.todo_green != self.done_green {
            let c = match (self.todo_red, self.todo_green) {
                (false, false) => LedColor::OFF,
                (false, true) => LedColor::GREEN,
                (true, false) => LedColor::RED,
                (true, true) => LedColor::ORANGE,
            };
            ev3rt::led_set_color(c);
            self.done_red = self.todo_red;
            self.done_green = self.todo_green;
        }
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub struct Duration {
    ticks: i32,
}

impl Duration {
    pub fn new(ticks: i32) -> Self {
        Self { ticks }
    }

    pub fn from_usec(usec: i32) -> Self {
        Self::new(usec)
    }

    pub fn from_msec(msec: i32) -> Self {
        Self::new(msec * 1000)
    }

    pub fn zero() -> Self {
        Self { ticks: 0 }
    }

    pub fn usec(self) -> i32 {
        self.ticks
    }

    pub fn msec(self) -> i32 {
        self.ticks / 1000
    }

    pub fn sec(self) -> i32 {
        self.ticks / 1000000
    }
}

impl core::ops::Add for Duration {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::new(self.ticks + other.ticks)
    }
}

impl core::ops::Sub for Duration {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(self.ticks - other.ticks)
    }
}

impl core::ops::AddAssign for Duration {
    fn add_assign(&mut self, other: Self) {
        self.ticks += other.ticks;
    }
}

impl core::ops::SubAssign for Duration {
    fn sub_assign(&mut self, other: Self) {
        self.ticks -= other.ticks;
    }
}

pub struct Time {
    ticks: u64,
    duration_from_last_read: Duration,
    duration_from_last_reset: Duration,
}

impl Time {
    pub fn new() -> Self {
        Self {
            ticks: ev3rt::get_utime(),
            duration_from_last_read: Duration::zero(),
            duration_from_last_reset: Duration::zero(),
        }
    }

    fn compute_duration_from_last_read(&self) -> (Duration, u64) {
        let previous = self.ticks;
        let current = ev3rt::get_utime();
        let delta = Duration::new((current - previous) as i32);
        (delta, current)
    }

    pub fn read(&mut self) {
        let (mut delta, mut current) = self.compute_duration_from_last_read();
        // It turns out that waiting "a bit" gives more reliable sensor reads...
        while delta.usec() < 800 {
            //ev3rt::msleep(1);
            let (d, c) = self.compute_duration_from_last_read();
            delta = d;
            current = c;
        }

        self.ticks = current;
        self.duration_from_last_read = delta;
        self.duration_from_last_reset += delta;
    }

    pub fn reset(&mut self) {
        self.duration_from_last_reset = Duration::zero();
    }

    pub fn from_last_read(&self) -> Duration {
        self.duration_from_last_read
    }

    pub fn from_last_reset(&self) -> Duration {
        self.duration_from_last_reset
    }
}

pub struct KeyStatus {
    from_last_event: Duration,
    last_state_duration: Duration,
    pressed: bool,
}

impl KeyStatus {
    pub fn new() -> Self {
        Self {
            from_last_event: Duration::zero(),
            last_state_duration: Duration::zero(),
            pressed: false,
        }
    }

    pub fn update(&mut self, pressed: bool, delta: Duration) {
        if self.pressed == pressed {
            self.from_last_event += delta;
        } else {
            self.pressed = pressed;
            self.last_state_duration = self.from_last_event;
            self.from_last_event = Duration::zero();
        }
    }

    pub fn is_pressed(&self) -> bool {
        self.pressed
    }

    pub fn is_released(&self) -> bool {
        !self.pressed
    }

    pub fn since_pressed(&self) -> Duration {
        if self.pressed {
            self.from_last_event
        } else {
            Duration::zero()
        }
    }

    pub fn since_released(&self) -> Duration {
        if !self.pressed {
            self.from_last_event
        } else {
            Duration::zero()
        }
    }

    pub fn last_state_duration(&self) -> Duration {
        self.last_state_duration
    }

    pub fn press_event(&self) -> bool {
        self.is_pressed() && self.from_last_event.ticks == 0
    }

    pub fn release_event(&self) -> bool {
        self.is_released() && self.from_last_event.ticks == 0
    }
}

pub struct Keys {
    pub up: KeyStatus,
    pub down: KeyStatus,
    pub left: KeyStatus,
    pub right: KeyStatus,
    pub enter: KeyStatus,
    pub back: KeyStatus,
}

impl Keys {
    pub fn new() -> Self {
        Self {
            up: KeyStatus::new(),
            down: KeyStatus::new(),
            left: KeyStatus::new(),
            right: KeyStatus::new(),
            enter: KeyStatus::new(),
            back: KeyStatus::new(),
        }
    }

    pub fn read(&mut self, delta: Duration, or: ScreenOrientation) {
        let (up, down, left, right, enter, back) = (
            button_is_pressed(Button::UP),
            button_is_pressed(Button::DOWN),
            button_is_pressed(Button::LEFT),
            button_is_pressed(Button::RIGHT),
            button_is_pressed(Button::ENTER),
            button_is_pressed(Button::BACK),
        );

        let (up, right, down, left) = match or {
            ScreenOrientation::Up => (up, right, down, left),
            ScreenOrientation::Right => (right, down, left, up),
            ScreenOrientation::Down => (down, left, up, right),
            ScreenOrientation::Left => (left, up, right, down),
        };

        self.up.update(up, delta);
        self.down.update(down, delta);
        self.left.update(left, delta);
        self.right.update(right, delta);
        self.enter.update(enter, delta);
        self.back.update(back, delta);
    }
}

pub struct Ev3 {
    pub sensors: [SensorData; 4],
    pub motors: [MotorData; 4],
    pub screen: Screen,
    pub leds: Leds,
    pub time: Time,
    pub keys: Keys,
}

pub trait SensorGetterMut {
    fn sensor(&mut self, port: SensorPort) -> &mut SensorData;
}
pub trait SensorGetter {
    fn sensor(&self, port: SensorPort) -> &SensorData;
}
impl SensorGetterMut for Ev3 {
    fn sensor(&mut self, port: SensorPort) -> &mut SensorData {
        &mut (self.sensors[port as usize])
    }
}
impl SensorGetter for Ev3 {
    fn sensor(&self, port: SensorPort) -> &SensorData {
        &(self.sensors[port as usize])
    }
}

pub trait MotorGetterMut {
    fn motor(&mut self, port: MotorPort) -> &mut MotorData;
}
pub trait MotorGetter {
    fn motor(&self, port: MotorPort) -> &MotorData;
}
impl MotorGetterMut for Ev3 {
    fn motor(&mut self, port: MotorPort) -> &mut MotorData {
        &mut (self.motors[port as usize])
    }
}
impl MotorGetter for Ev3 {
    fn motor(&self, port: MotorPort) -> &MotorData {
        &(self.motors[port as usize])
    }
}

impl Ev3 {
    pub fn new() -> Ev3 {
        Ev3 {
            sensors: [
                SensorData::new(SensorPort::S1),
                SensorData::new(SensorPort::S2),
                SensorData::new(SensorPort::S3),
                SensorData::new(SensorPort::S4),
            ],
            motors: [
                MotorData::new(MotorPort::A),
                MotorData::new(MotorPort::B),
                MotorData::new(MotorPort::C),
                MotorData::new(MotorPort::D),
            ],
            screen: Screen::new(),
            leds: Leds::new(),
            time: Time::new(),
            keys: Keys::new(),
        }
    }

    pub fn s1(&mut self) -> &mut SensorData {
        &mut (self.sensors[0])
    }
    pub fn s2(&mut self) -> &mut SensorData {
        &mut (self.sensors[1])
    }
    pub fn s3(&mut self) -> &mut SensorData {
        &mut (self.sensors[2])
    }
    pub fn s4(&mut self) -> &mut SensorData {
        &mut (self.sensors[3])
    }

    pub fn ma(&mut self) -> &mut MotorData {
        &mut (self.motors[0])
    }
    pub fn mb(&mut self) -> &mut MotorData {
        &mut (self.motors[1])
    }
    pub fn mc(&mut self) -> &mut MotorData {
        &mut (self.motors[2])
    }
    pub fn md(&mut self) -> &mut MotorData {
        &mut (self.motors[3])
    }

    pub fn lcd_clear(&self) {
        lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LcdColor::WHITE);
    }

    pub fn reset(&self) {
        motor_stop(MotorPort::A, false);
        motor_stop(MotorPort::B, false);
        motor_stop(MotorPort::C, false);
        motor_stop(MotorPort::D, false);
        sensor_config(SensorPort::S1, SensorType::NONE);
        sensor_config(SensorPort::S2, SensorType::NONE);
        sensor_config(SensorPort::S3, SensorType::NONE);
        sensor_config(SensorPort::S4, SensorType::NONE);
        led_set_color(LedColor::OFF);
        lcd_set_font(LcdFont::MEDIUM);
        self.lcd_clear();
        led_set_color(LedColor::OFF);
    }

    pub fn apply_configuration(&mut self) {
        self.leds.reset();
        self.lcd_clear();

        self.screen.setup_info_area_full();
        self.screen.set_info_count(9);
        self.screen.setup_info(0, 6, 1, 3, 1, 1);
        self.screen.setup_info(1, 2, 2, 3, 1, 4);
        self.screen.setup_info(2, 2, 2, 3, 2, 4);
        self.screen.setup_info(3, 2, 2, 3, 3, 4);
        self.screen.setup_info(4, 2, 2, 3, 4, 4);
        self.screen.setup_info(5, 2, 3, 3, 1, 4);
        self.screen.setup_info(6, 2, 3, 3, 2, 4);
        self.screen.setup_info(7, 2, 3, 3, 3, 4);
        self.screen.setup_info(8, 2, 3, 3, 4, 4);
        self.screen
            .setup_info_glyphs(0, &[Gph::C, Gph::O, Gph::N, Gph::F, Gph::I, Gph::G]);
        self.screen.setup_info_glyphs(1, &[Gph::S, Gph::V1]);
        self.screen.setup_info_glyphs(2, &[Gph::S, Gph::V2]);
        self.screen.setup_info_glyphs(3, &[Gph::S, Gph::V3]);
        self.screen.setup_info_glyphs(4, &[Gph::S, Gph::V4]);
        self.screen.setup_info_glyphs(5, &[Gph::M, Gph::A]);
        self.screen.setup_info_glyphs(6, &[Gph::M, Gph::B]);
        self.screen.setup_info_glyphs(7, &[Gph::M, Gph::C]);
        self.screen.setup_info_glyphs(8, &[Gph::M, Gph::D]);
        for s in 0..3usize {
            if self.sensors[s].cfg == SensorConfiguration::None {
                self.screen
                    .setup_info_glyphs(1 + s, &[Gph::Minus, Gph::Minus]);
            }
            if self.motors[s].cfg == MotorType::NONE {
                self.screen
                    .setup_info_glyphs(5 + s, &[Gph::Minus, Gph::Minus]);
            }
        }

        self.screen.set_info_bold(0, true);
        self.screen.render_info();
        self.ma().attempt_cfg_apply();
        self.mb().attempt_cfg_apply();
        self.mc().attempt_cfg_apply();
        self.md().attempt_cfg_apply();
        self.screen.set_info_bold(0, false);
        self.screen.render_info();

        const TEST_LENGTH: i32 = 1000;
        const TEST_MOVEMENT: i32 = 800;
        const TEST_PERIODS: i32 = 8;
        const TEST_PERIOD: i32 = TEST_MOVEMENT / TEST_PERIODS;

        self.time.reset();
        while self.time.duration_from_last_reset.msec() < TEST_LENGTH * 4
            || (!self.s1().cfg_applied)
            || (!self.s2().cfg_applied)
            || (!self.s3().cfg_applied)
            || (!self.s4().cfg_applied)
        {
            self.screen.deselect_infos();
            let millis = self.time.duration_from_last_reset.msec();

            let current_motor = (millis / TEST_LENGTH) as usize;
            if current_motor < 4 {
                if current_motor > 0 {
                    motor_set_power(self.motors[current_motor - 1].port(), 0);
                }
                self.screen.set_info_bold(5 + current_motor, true);
                let mut power = 0;
                if self.motors[current_motor].cfg != MotorType::NONE {
                    let elapsed_millis = millis % TEST_LENGTH;
                    power = if elapsed_millis < TEST_MOVEMENT {
                        const TEST_POWER: i32 = 30;
                        let period = (elapsed_millis / (TEST_PERIOD / 4)) % 4;
                        match period {
                            0 | 3 => TEST_POWER,
                            1 | 2 => -TEST_POWER,
                            _ => 0,
                        }
                    } else {
                        0
                    }
                }
                motor_set_power(self.motors[current_motor].port(), power);
            } else {
                self.stop();
            }

            for current_sensor in 0..4usize {
                let todo = !self.sensors[current_sensor].configuration_applied();
                self.screen.set_info_bold(1 + current_sensor, todo);
                if todo {
                    self.sensors[current_sensor].attempt_cfg_apply();
                }
            }

            self.screen.render_info();
            self.time.read();
        }
        self.screen.set_info_count(0);
        self.screen.render_info();
        self.lcd_clear();
    }

    pub fn stop(&mut self) {
        self.ma().stop(false);
        self.mb().stop(false);
        self.mc().stop(false);
        self.md().stop(false);
    }

    pub fn read(&mut self) {
        self.s1().read();
        self.s2().read();
        self.s3().read();
        self.s4().read();
        self.ma().read();
        self.mb().read();
        self.mc().read();
        self.md().read();
        self.time.read();
        self.keys
            .read(self.time.duration_from_last_read, self.screen.orientation())
    }

    pub fn apply(&mut self) {
        self.ma().apply_power();
        self.mb().apply_power();
        self.mc().apply_power();
        self.md().apply_power();
        self.leds.apply();
        self.screen.render_info();
    }
}

// sin(v) * 1000, with x in deg
pub fn sin(v: i32) -> i32 {
    let v = v / 15;

    if v >= 0 {
        SIN_TABLE[(v % 24) as usize]
    } else {
        SIN_TABLE[((-v) % 24) as usize]
    }
}

// cos(v) * 1000, with x in deg
pub fn cos(v: i32) -> i32 {
    sin(v + 90)
}
