//! A drone as one object in time: a constant-velocity Kalman filter on a flat plane in km.
//! Between posts it flies on and grows less certain; a post is a noisy observation of where
//! it is and which way it flies, weighed against what the filter already knows. Posts that
//! run ahead of the drone (a panic: "на Вороновицю" 75 s after "на Немирів") move it only as
//! far as that knowledge allows; after a long silence a post sets it almost entirely.

type M4 = [[f64; 4]; 4];

/// Position (km east, km north) and velocity (km/h east, km/h north), with covariance.
#[derive(Debug, Clone)]
pub struct Filter {
    x: [f64; 4],
    p: M4,
}

/// How noisy the motion and the posts are.
#[derive(Debug, Clone, Copy)]
pub struct Noise {
    /// Random acceleration of the drone, (km/h)² per hour of flight: how much it may turn or
    /// change pace between posts.
    pub accel: f64,
    /// Where "на X" puts the drone, σ along its way to X (L and the post's timing) and across.
    pub along_km: f64,
    pub cross_km: f64,
    /// "на X" says it flies towards X at cruise speed, σ of each velocity component, km/h.
    pub velocity_kmh: f64,
}

impl Filter {
    /// A drone first seen at `pos` flying `vel`, with these σs.
    pub fn new(pos: (f64, f64), vel: (f64, f64), pos_km: f64, vel_kmh: f64) -> Self {
        let mut p = [[0.0; 4]; 4];
        p[0][0] = pos_km * pos_km;
        p[1][1] = pos_km * pos_km;
        p[2][2] = vel_kmh * vel_kmh;
        p[3][3] = vel_kmh * vel_kmh;
        Self {
            x: [pos.0, pos.1, vel.0, vel.1],
            p,
        }
    }

    pub fn pos(&self) -> (f64, f64) {
        (self.x[0], self.x[1])
    }

    pub fn vel(&self) -> (f64, f64) {
        (self.x[2], self.x[3])
    }

    /// σ² of the position along the unit direction `u`.
    pub fn pos_var_along(&self, u: (f64, f64)) -> f64 {
        let p = &self.p;
        u.0 * u.0 * p[0][0] + 2.0 * u.0 * u.1 * p[0][1] + u.1 * u.1 * p[1][1]
    }

    /// σ² of the velocity along `u`.
    pub fn vel_var_along(&self, u: (f64, f64)) -> f64 {
        let p = &self.p;
        u.0 * u.0 * p[2][2] + 2.0 * u.0 * u.1 * p[2][3] + u.1 * u.1 * p[3][3]
    }

    /// Flies on for `hours`.
    pub fn predict(&mut self, hours: f64, noise: &Noise) {
        let dt = hours.max(0.0);
        self.x[0] += self.x[2] * dt;
        self.x[1] += self.x[3] * dt;
        let mut f = identity();
        f[0][2] = dt;
        f[1][3] = dt;
        let mut p = mul(&mul(&f, &self.p), &transpose(&f));
        let q = noise.accel;
        let (a, b, c) = (q * dt.powi(3) / 3.0, q * dt * dt / 2.0, q * dt);
        for axis in 0..2 {
            p[axis][axis] += a;
            p[axis][axis + 2] += b;
            p[axis + 2][axis] += b;
            p[axis + 2][axis + 2] += c;
        }
        self.p = p;
    }

    /// A post says the drone is at `pos` (σ `along` in the direction `u`, `cross` across it)
    /// and, if `vel` is given, flies at `vel` (σ `vel_sigma` per component).
    pub fn observe(
        &mut self,
        pos: (f64, f64),
        u: (f64, f64),
        along: f64,
        cross: f64,
        vel: Option<((f64, f64), f64)>,
    ) {
        // Observation of the whole state: an unknown velocity gets a huge σ.
        let z = match vel {
            Some((v, _)) => [pos.0, pos.1, v.0, v.1],
            None => [pos.0, pos.1, self.x[2], self.x[3]],
        };
        let mut r = [[0.0; 4]; 4];
        let (a2, c2) = (along * along, cross * cross);
        // Rotate diag(along², cross²) into the plane: along u = (ux, uy), across (-uy, ux).
        r[0][0] = a2 * u.0 * u.0 + c2 * u.1 * u.1;
        r[1][1] = a2 * u.1 * u.1 + c2 * u.0 * u.0;
        r[0][1] = (a2 - c2) * u.0 * u.1;
        r[1][0] = r[0][1];
        let v2 = vel.map_or(1e12, |(_, s)| s * s);
        r[2][2] = v2;
        r[3][3] = v2;
        let s = add(&self.p, &r);
        let Some(si) = inverse(&s) else {
            return;
        };
        let k = mul(&self.p, &si);
        let innov: Vec<f64> = (0..4).map(|i| z[i] - self.x[i]).collect();
        for (x, row) in self.x.iter_mut().zip(&k) {
            *x += row.iter().zip(&innov).map(|(a, b)| a * b).sum::<f64>();
        }
        let ik = sub(&identity(), &k);
        self.p = mul(&ik, &self.p);
    }
}

fn identity() -> M4 {
    let mut m = [[0.0; 4]; 4];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

fn mul(a: &M4, b: &M4) -> M4 {
    let mut m = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    m
}

fn transpose(a: &M4) -> M4 {
    let mut m = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[j][i];
        }
    }
    m
}

fn add(a: &M4, b: &M4) -> M4 {
    let mut m = *a;
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] += b[i][j];
        }
    }
    m
}

fn sub(a: &M4, b: &M4) -> M4 {
    let mut m = *a;
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] -= b[i][j];
        }
    }
    m
}

/// Gauss–Jordan inverse; None if singular.
fn inverse(a: &M4) -> Option<M4> {
    let mut m = *a;
    let mut inv = identity();
    for c in 0..4 {
        let p = (c..4).max_by(|&i, &j| m[i][c].abs().total_cmp(&m[j][c].abs()))?;
        if m[p][c].abs() < 1e-12 {
            return None;
        }
        m.swap(c, p);
        inv.swap(c, p);
        let pivot = m[c][c];
        for j in 0..4 {
            m[c][j] /= pivot;
            inv[c][j] /= pivot;
        }
        for i in 0..4 {
            if i != c {
                let f = m[i][c];
                for j in 0..4 {
                    m[i][j] -= f * m[c][j];
                    inv[i][j] -= f * inv[c][j];
                }
            }
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOISE: Noise = Noise {
        accel: 2.0e5,
        along_km: 15.0,
        cross_km: 8.0,
        velocity_kmh: 120.0,
    };

    /// A post right after the last one barely moves a well-known drone; after a long silence
    /// the same post nearly sets it.
    #[test]
    fn trust_in_a_post_grows_with_silence() {
        let jump = |minutes: f64| {
            let mut f = Filter::new((0.0, -60.0), (0.0, 425.0), 5.0, 30.0);
            f.predict(minutes / 60.0, &NOISE);
            let flown = f.pos().1;
            // The post: 30 km further on than dead reckoning has it.
            f.observe(
                (0.0, flown + 30.0),
                (0.0, 1.0),
                NOISE.along_km,
                NOISE.cross_km,
                None,
            );
            f.pos().1 - flown
        };
        assert!(jump(1.0) < 10.0, "{}", jump(1.0));
        assert!(jump(15.0) > 20.0, "{}", jump(15.0));
    }
}
