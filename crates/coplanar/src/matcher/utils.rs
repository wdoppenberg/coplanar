use alloc::vec::Vec;
use itertools::Itertools;
use nalgebra as na;

use crate::Error;
use crate::PlanarEllipse;
use crate::geom::ellipse::repr::EllipseRepr;

/// Returns if a collection of ellipse (centers) is ordered clockwise (true) or not (false)
pub(crate) fn is_clockwise<'a, F, I, R>(ellipses: I) -> Result<bool, Error>
where
    F: na::RealField,
    I: IntoIterator<Item = &'a PlanarEllipse<R>>,
    R: EllipseRepr<F = F> + 'a,
{
    let s = ellipses
        .into_iter()
        // Get ellipse centers
        .filter_map(|e| e.center().ok())
        // Get sliding window iterator with n=2
        .tuple_windows()
        // Calculate "area"
        .map(|((x1, y1), (x2, y2))| (x2 - x1) * (y2 + y1))
        // Sum
        .fold(F::zero(), |acc, x| acc + x);

    Ok(s > F::zero())
}

use thiserror::Error as ThisError;

#[derive(ThisError, Debug)]
pub enum EpsError {
    #[error("Number of detections must be equal or higher than 3")]
    TooFewDetections,
}

/// Generator function returning next triad according to Enhanced Pattern Shifting Method from
/// Arnas, D., Fialho, M. A. A., & Mortari, D. (2017). Fast and robust kernel generators for star
/// trackers. Acta Astronautica, 134 (August 2016), 291–302.
/// https://doi.org/10.1016/j.actaastro.2017.02.016
#[allow(dead_code)]
pub struct EnhancedPatternShifting<T> {
    items: Vec<T>,
    n: usize,
    dj: usize,
    dk: usize,
    ii: usize,
    i: usize,
}

impl<T> EnhancedPatternShifting<T> {
    #[allow(dead_code)]
    pub fn new<I>(iter: I, n: usize, _start_n: usize) -> Result<Self, EpsError>
    where
        I: Iterator<Item = T>,
    {
        if n < 3 {
            return Err(EpsError::TooFewDetections);
        }
        let items: Vec<T> = iter.take(n).collect();

        Ok(Self {
            items,
            n,
            dj: 1,
            dk: 1,
            ii: 1,
            i: 1,
        })
    }
}

impl<T> Iterator for EnhancedPatternShifting<T>
where
    T: Copy,
{
    type Item = (T, T, T);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.dj > self.n - 2 {
                return None;
            }

            if self.dk > self.n - self.dj - 1 {
                self.dk = 1;
                self.dj += 1;
                self.ii = 1;
                self.i = 1;
                continue;
            }

            if self.ii > 3 {
                self.ii = 1;
                self.dk += 1;
                self.i = 1;
                continue;
            }

            if self.i > self.n - self.dj - self.dk {
                self.ii += 1;
                self.i = self.ii;
                continue;
            }

            let i_idx = self.i;
            let j_idx = i_idx + self.dj;
            let k_idx = j_idx + self.dk;

            let result = (
                self.items[i_idx - 1],
                self.items[j_idx - 1],
                self.items[k_idx - 1],
            );

            self.i += 3;

            return Some(result);
        }
    }
}

// Extension trait for easy usage
#[allow(dead_code)]
pub trait EnhancedPatternShiftingExt: Iterator + Sized {
    fn enhanced_pattern_shift(
        self,
        n: usize,
        start_n: usize,
    ) -> Result<EnhancedPatternShifting<Self::Item>, EpsError> {
        EnhancedPatternShifting::new(self, n, start_n)
    }
}

impl<T: Iterator> EnhancedPatternShiftingExt for T {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PlanarEllipse;
    use alloc::collections::BTreeSet;
    use alloc::vec;

    #[test]
    fn test_simple_case_cw() -> Result<(), Error> {
        let e1 = &PlanarEllipse::from_parameters(1., 1., 0., 0., 0.);
        let e2 = &PlanarEllipse::from_parameters(1., 1., 0., 0., 1.);
        let e3 = &PlanarEllipse::from_parameters(1., 1., 0., 1., 0.);

        assert!(is_clockwise([e1, e2, e3])?);

        Ok(())
    }

    #[test]
    fn test_simple_case_ccw() -> Result<(), Error> {
        let e1 = &PlanarEllipse::from_parameters(1., 1., 0., 0., 0.);
        let e2 = &PlanarEllipse::from_parameters(1., 1., 0., 1., 0.);
        let e3 = &PlanarEllipse::from_parameters(1., 1., 0., 0., 1.);

        assert!(!is_clockwise([e1, e2, e3])?);

        Ok(())
    }

    #[test]
    fn test_worst_case_failure_count() {
        let n = 14; // As used in Fig. 4 of the paper
        let data: Vec<_> = (0..n).collect();
        let mut max_failures = 0;

        // Test for different numbers of false stars
        for f in 1..=(n - 3) {
            let mut failure_count = 0;
            // Find worst case by trying different false star positions
            let kernels = data.iter().enhanced_pattern_shift(n, 0).unwrap();

            for (a, b, c) in kernels {
                failure_count += 1;
                // Check if kernel contains no false stars
                if !contains_false_stars((a, b, c), &data, f) {
                    break;
                }
            }
            max_failures = max_failures.max(failure_count);
        }

        // Should be lower than the theoretical maximum from lexicographic ordering
        assert!(max_failures < n * (n - 1) * (n - 2) / 6);
    }

    #[test]
    fn test_max_index_distribution() {
        let n = 7; // Example from paper
        let data: Vec<_> = (0..n).collect();
        let mut counts = vec![0; n];

        let triads: Vec<_> = data.iter().enhanced_pattern_shift(n, 0).unwrap().collect();

        // Check if all kernels are generated
        assert_eq!(
            triads.len(),
            n * (n - 1) * (n - 2) / 6,
            "Should generate all combinations"
        );

        let mut max_diff_ever = 0;
        for (a, b, c) in &triads {
            counts[**a] += 1;
            counts[**b] += 1;
            counts[**c] += 1;
            let current_diff = counts.iter().max().unwrap() - counts.iter().min().unwrap();
            max_diff_ever = max_diff_ever.max(current_diff);
        }

        assert_eq!(max_diff_ever, 6);
    }

    #[test]
    fn test_expected_time_to_discovery() {
        let n = 10; // Use a smaller n to keep test fast, Table 4 says T(Q) = 18.03 for n=10
        let data: Vec<_> = (0..n).collect();

        let mut total_kernels = 0;
        let mut total_scenes = 0;

        // The paper says |S| = sum_{nt=k}^n (n choose nt)
        // This is all combinations of stars where at least k stars are present.
        for nt in 3..=n {
            for real_indices in (0..n).combinations(nt) {
                let mut count = 0;
                for (a, b, c) in data.iter().enhanced_pattern_shift(n, 0).unwrap() {
                    count += 1;
                    if real_indices.contains(a)
                        && real_indices.contains(b)
                        && real_indices.contains(c)
                    {
                        break;
                    }
                }
                total_kernels += count;
                total_scenes += 1;
            }
        }

        let avg_time = total_kernels as f64 / total_scenes as f64;
        // Table 4: n=10, EPS, k=3 -> 18.03
        assert!((avg_time - 18.03).abs() < 1.0);
    }

    #[test]
    fn test_sequence_properties() {
        let n = 5;
        let data: Vec<_> = (0..n).collect();
        let triads: Vec<_> = data.iter().enhanced_pattern_shift(n, 0).unwrap().collect();

        // Test no duplicates
        let mut seen = BTreeSet::new();
        for &(a, b, c) in &triads {
            let key = (a, b, c);
            assert!(seen.insert(key), "Found duplicate triad");
        }

        // Test ordering property from EPS algorithm:
        // Elements increment by 3 where possible
        for window in triads.windows(2) {
            if let [(a1, _, _), (a2, _, _)] = window {
                // Either the difference is 3 or we've wrapped/changed pattern
                if a2 > a1 {
                    assert!(
                        *a2 - *a1 == 3 || *a2 - *a1 == 1,
                        "Invalid increment pattern"
                    );
                }
            }
        }
    }

    // Helper function to simulate false stars
    fn contains_false_stars<T: PartialEq>(
        (i, j, k): (&T, &T, &T),
        data: &Vec<T>,
        false_stars: usize,
    ) -> bool {
        let false_start = data.len() - false_stars;
        let idx_i = data.iter().position(|x| x == i).unwrap();
        let idx_j = data.iter().position(|x| x == j).unwrap();
        let idx_k = data.iter().position(|x| x == k).unwrap();

        idx_i >= false_start || idx_j >= false_start || idx_k >= false_start
    }
}
