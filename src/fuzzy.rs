use std::{
    cmp::Ordering,
    ops::{BitAnd, BitOr, Not},
};

pub enum FuzzySet {
    Rising(f64, f64),
    Falling(f64, f64),
    Triangle(f64, f64, f64),
    Trapezoid(f64, f64, f64, f64),
}

fn fmin(a: f64, b: f64) -> f64 {
    match a.partial_cmp(&b).unwrap_or(Ordering::Equal) {
        Ordering::Less => a,
        _ => b,
    }
}

fn fmax(a: f64, b: f64) -> f64 {
    match a.partial_cmp(&b).unwrap_or(Ordering::Equal) {
        Ordering::Greater => a,
        _ => b,
    }
}

fn ascend(start: f64, value: f64, end: f64) -> f64 {
    (value - start) / (end - start)
}

fn descend(start: f64, value: f64, end: f64) -> f64 {
    (end - value) / (end - start)
}

impl FuzzySet {
    pub fn fuzzify(&self, value: f64) -> anyhow::Result<FuzzyVar> {
        let result = match self {
            FuzzySet::Rising(start, end) => {
                if *start < *end {
                    ascend(*start, value, *end)
                } else {
                    anyhow::bail!("{start} is not less than {end}")
                }
            }
            FuzzySet::Falling(start, end) => {
                if *start < *end {
                    descend(*start, value, *end)
                } else {
                    anyhow::bail!("{start} is not less than {end}")
                }
            }
            FuzzySet::Triangle(start, peak, end) => {
                if *start < *peak && *peak < *end {
                    if value < *peak {
                        ascend(*start, value, *peak)
                    } else {
                        descend(*peak, value, *end)
                    }
                } else {
                    anyhow::bail!("{start}, {peak}, and {end} are not ascending")
                }
            }
            FuzzySet::Trapezoid(start, peak_start, peak_end, end) => {
                if *start < *peak_start && *peak_start < *peak_end && *peak_end < *end {
                    if value < *peak_start {
                        ascend(*start, value, *peak_start)
                    } else if value < *peak_end {
                        1.0
                    } else {
                        descend(*peak_end, value, *end)
                    }
                } else {
                    anyhow::bail!("{start}, {peak_start}, {peak_end}, and {end} are not ascending")
                }
            }
        };
        Ok(FuzzyVar::new(result))
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug)]
pub struct FuzzyVar(f64);

impl FuzzyVar {
    pub fn new(value: f64) -> Self {
        Self(fmin(1.0, fmax(0.0, value)))
    }

    pub fn defuzzify(&self, zero: f64, one: f64) -> f64 {
        zero + self.0 * (one - zero)
    }
}

impl BitAnd for FuzzyVar {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        Self::new(fmin(self.0, rhs.0))
    }
}

impl BitOr for FuzzyVar {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self::new(fmax(self.0, rhs.0))
    }
}

impl Not for FuzzyVar {
    type Output = Self;

    fn not(self) -> Self::Output {
        Self::new(1.0 - self.0)
    }
}

#[cfg(test)]
mod tests {
    use crate::fuzzy::FuzzySet;

    macro_rules! fz {
        ($v:expr) => {
            crate::fuzzy::FuzzyVar::new($v)
        };
    }

    #[test]
    fn test_and_or_not() {
        assert_eq!(fz!(0.75), fz!(1.0) & fz!(0.75));
        assert_eq!(fz!(1.0), fz!(1.0) | fz!(0.75));
        assert_eq!(fz!(1.0), !fz!(0.0));
        assert_eq!(fz!(0.0), !fz!(1.0));
    }

    #[test]
    fn test_fuzzify_rising() {
        for (expected, height) in [
            (1.0, 76),
            (0.75, 74),
            (0.5, 72),
            (0.25, 70),
            (0.0, 68),
            (1.0, 80),
            (0.0, 62),
        ] {
            let f = FuzzySet::Rising(68.0, 76.0).fuzzify(height as f64).unwrap();
            assert_eq!(fz!(expected), f);
        }
    }

    #[test]
    fn test_defuzzify() {
        for (inseam_size, fuzzy_height) in
            [(36.0, 1.0), (34.5, 0.75), (33.0, 0.5), (31.5, 0.25), (30.0, 0.0)]
        {
            let fuzzy_height = fz!(fuzzy_height);
            assert_eq!(inseam_size, fuzzy_height.defuzzify(30.0, 36.0))
        }
    }
}
