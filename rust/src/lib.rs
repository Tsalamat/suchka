//! Детектор дорожной разметки v2. Весь расчёт — на CPU телефона.
//!
//! Конвейер:
//!   Y-плоскость -> уменьшение -> фильтр "узкая светлая полоса" (учитывает перспективу)
//!   -> преобразование Хафа -> реальные отрезки -> оставляем только линии,
//!   сходящиеся в одной точке схода (как настоящая разметка) -> сплошная/прерывистая.
//!
//! Отрезки строятся ТОЛЬКО из найденных пикселей в текущем кадре.
//! Никакого продолжения линий, заливки полосы и памяти между кадрами.

use std::f32::consts::PI;

const TARGET_WIDTH: usize = 480;
const MIN_VOTES: u16 = 18;
const MAX_PEAKS: usize = 24;
const LINE_TOL: f32 = 1.5;
const MIN_SEG_LEN: f32 = 10.0;
const TAU_MAX_FRAC: f32 = 0.03; // ширина разметки у нижнего края кадра (доля ширины)
const HORIZON: f32 = 0.40; // где примерно горизонт (доля высоты)
const VP_TOL_FRAC: f32 = 0.035; // допуск прохождения линии через точку схода

/// Область поиска (доли кадра). Должна совпадать с рисованием в Dart.
pub const ROI_TOP: f32 = 0.42;
pub const ROI_BOTTOM: f32 = 0.96;
pub const ROI_TOP_LEFT: f32 = 0.20;
pub const ROI_TOP_RIGHT: f32 = 0.80;

pub const SEG_STRIDE: usize = 5; // x1,y1,x2,y2,kind (kind: 0 = прерывистая, 1 = сплошная)

#[inline]
fn in_roi(x: usize, y: usize, w: usize, h: usize) -> bool {
    let top = ROI_TOP * h as f32;
    let bottom = ROI_BOTTOM * h as f32;
    let yf = y as f32;
    if yf < top || yf > bottom {
        return false;
    }
    let t = (yf - top) / (bottom - top).max(1.0);
    let left = ROI_TOP_LEFT * (1.0 - t) * w as f32;
    let right = (ROI_TOP_RIGHT + (1.0 - ROI_TOP_RIGHT) * t) * w as f32;
    (x as f32) >= left && (x as f32) <= right
}

fn downscale(src: &[u8], w: usize, h: usize, stride: usize) -> (Vec<u8>, usize, usize, usize) {
    let f = ((w + TARGET_WIDTH - 1) / TARGET_WIDTH).max(1);
    let (nw, nh) = (w / f, h / f);
    let mut out = vec![0u8; nw * nh];
    for y in 0..nh {
        for x in 0..nw {
            let mut sum = 0u32;
            for dy in 0..f {
                let row = (y * f + dy) * stride + x * f;
                for dx in 0..f {
                    sum += src[row + dx] as u32;
                }
            }
            out[y * nw + x] = (sum / (f * f) as u32) as u8;
        }
    }
    (out, nw, nh, f)
}

/// Фильтр "светлая узкая полоса на более тёмном фоне" (lane-marking filter).
/// Реагирует на штрихи шириной ~tau и НЕ реагирует на ступенчатые границы
/// (тени, бордюры, края асфальта) и на широкие однородные области.
fn ridge_points(img: &[u8], w: usize, h: usize, thr: i32) -> Vec<(f32, f32)> {
    // лёгкое вертикальное сглаживание (подавляет шум, не размывая полосу по горизонтали)
    let mut sm = vec![0i16; w * h];
    for y in 0..h {
        let (u, d) = (y.saturating_sub(1), (y + 1).min(h - 1));
        for x in 0..w {
            sm[y * w + x] =
                ((img[u * w + x] as i16) + 2 * (img[y * w + x] as i16) + (img[d * w + x] as i16)) / 4;
        }
    }
    let hy = HORIZON * h as f32;
    let top = (ROI_TOP * h as f32) as usize;
    let bottom = ((ROI_BOTTOM * h as f32) as usize).min(h - 1);
    let mut pts = Vec::new();
    let mut resp = vec![0i16; w];
    for y in top..=bottom {
        let tau = ((TAU_MAX_FRAC * w as f32 * (y as f32 - hy) / (h as f32 - hy)).round() as isize).max(1) as usize;
        if 2 * tau + 3 >= w {
            continue;
        }
        let row = &sm[y * w..(y + 1) * w];
        for v in resp.iter_mut() {
            *v = 0;
        }
        for x in tau..w - tau {
            let (l, c, r) = (row[x - tau], row[x], row[x + tau]);
            let v = 2 * c - l - r - (l - r).abs();
            resp[x] = v.max(0);
        }
        for x in tau + 1..w - tau - 1 {
            let v = resp[x];
            if v as i32 >= thr && v > resp[x - 1] && v >= resp[x + 1] && in_roi(x, y, w, h) {
                pts.push((x as f32, y as f32));
            }
        }
    }
    pts
}

struct Line {
    rho: f32,
    c: f32,
    s: f32,
    segs: Vec<[f32; 4]>,
    weight: f32,
    dashed: bool,
}

fn build_lines(pts: &[(f32, f32)], sw: usize, sh: usize) -> Vec<Line> {
    let thetas: Vec<usize> = (10..=65).chain(115..=170).collect();
    let trig: Vec<(f32, f32)> = thetas
        .iter()
        .map(|d| {
            let a = *d as f32 * PI / 180.0;
            (a.cos(), a.sin())
        })
        .collect();
    let diag = ((sw * sw + sh * sh) as f32).sqrt().ceil() as i32;
    let nrho = (2 * diag + 1) as usize;
    let nt = thetas.len();
    let mut acc = vec![0u16; nt * nrho];
    for &(x, y) in pts {
        for (ti, (c, s)) in trig.iter().enumerate() {
            let rho = (x * c + y * s).round() as i32 + diag;
            acc[ti * nrho + rho as usize] += 1;
        }
    }
    let mut peaks: Vec<(u16, usize, usize)> = Vec::new();
    for ti in 0..nt {
        for ri in 0..nrho {
            let v = acc[ti * nrho + ri];
            if v < MIN_VOTES {
                continue;
            }
            let mut is_max = true;
            'win: for dt in -3i32..=3 {
                for dr in -4i32..=4 {
                    if dt == 0 && dr == 0 {
                        continue;
                    }
                    let (tt, rr) = (ti as i32 + dt, ri as i32 + dr);
                    if tt < 0 || rr < 0 || tt >= nt as i32 || rr >= nrho as i32 {
                        continue;
                    }
                    let o = acc[tt as usize * nrho + rr as usize];
                    if o > v || (o == v && (dt < 0 || (dt == 0 && dr < 0))) {
                        is_max = false;
                        break 'win;
                    }
                }
            }
            if is_max {
                peaks.push((v, ti, ri));
            }
        }
    }
    peaks.sort_by(|a, b| b.0.cmp(&a.0));
    peaks.truncate(MAX_PEAKS);

    let top = ROI_TOP * sh as f32;
    let span = ((ROI_BOTTOM - ROI_TOP) * sh as f32).max(1.0);
    let mut used = vec![false; pts.len()];
    let mut lines = Vec::new();

    for (_, ti, ri) in peaks {
        let (c, s) = trig[ti];
        let rho = (ri as i32 - diag) as f32;
        let mut on: Vec<(f32, usize)> = Vec::new();
        for (i, &(x, y)) in pts.iter().enumerate() {
            if !used[i] && (x * c + y * s - rho).abs() <= LINE_TOL {
                on.push((-x * s + y * c, i));
            }
        }
        if on.len() < 8 {
            continue;
        }
        on.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // Разбиваем на непрерывные участки. Допустимый разрыв растёт к низу кадра (перспектива).
        let mut runs: Vec<(usize, usize)> = Vec::new();
        let mut start = 0usize;
        for k in 1..=on.len() {
            let brk = k == on.len() || {
                let py = pts[on[k - 1].1].1;
                let gap = 3.0 + 6.0 * ((py - top) / span).clamp(0.0, 1.0);
                on[k].0 - on[k - 1].0 > gap
            };
            if brk {
                if on[k - 1].0 - on[start].0 >= MIN_SEG_LEN && k - start >= 5 {
                    runs.push((start, k - 1));
                }
                start = k;
            }
        }
        if runs.is_empty() {
            continue;
        }
        let mut segs = Vec::new();
        let (mut weight, mut longest) = (0.0f32, 0.0f32);
        for &(a, b) in &runs {
            let (x0, y0) = pts[on[a].1];
            let (x1, y1) = pts[on[b].1];
            let len = on[b].0 - on[a].0;
            weight += len;
            longest = longest.max(len);
            segs.push([x0, y0, x1, y1]);
            for m in a..=b {
                used[on[m].1] = true;
            }
        }
        let dashed = runs.len() >= 2 || longest < 0.2 * sh as f32;
        lines.push(Line { rho, c, s, segs, weight, dashed });
    }
    lines
}

/// Оставляет линии, которые сходятся в общей точке схода (признак реальной дорожной разметки).
/// Если согласованной пары нет — консервативный запасной режим по наклону.
fn select_lines(lines: Vec<Line>, sw: usize, sh: usize) -> Vec<Line> {
    let tol = VP_TOL_FRAC * sw as f32;
    let (mut best_score, mut best_vp): (f32, Option<(f32, f32)>) = (0.0, None);
    for i in 0..lines.len() {
        for j in i + 1..lines.len() {
            let (a, b) = (&lines[i], &lines[j]);
            let det = a.c * b.s - b.c * a.s;
            if det.abs() < 0.14 {
                continue; // почти параллельны на изображении
            }
            let vx = (a.rho * b.s - b.rho * a.s) / det;
            let vy = (a.c * b.rho - b.c * a.rho) / det;
            if vy < -0.3 * sh as f32 || vy > ROI_TOP * sh as f32 + 0.1 * sh as f32 {
                continue;
            }
            if vx < -0.5 * sw as f32 || vx > 1.5 * sw as f32 {
                continue;
            }
            let score: f32 = lines
                .iter()
                .filter(|l| (vx * l.c + vy * l.s - l.rho).abs() <= tol)
                .map(|l| l.weight)
                .sum();
            if score > best_score {
                best_score = score;
                best_vp = Some((vx, vy));
            }
        }
    }
    let mut kept: Vec<Line> = match best_vp {
        Some((vx, vy)) => lines
            .into_iter()
            .filter(|l| (vx * l.c + vy * l.s - l.rho).abs() <= tol)
            .collect(),
        None => lines
            .into_iter()
            .filter(|l| {
                // Левая линия идёт "/" (s*c>0), правая "\" (s*c<0).
                let (mut mx, mut n) = (0.0f32, 0.0f32);
                for sg in &l.segs {
                    mx += sg[0] + sg[2];
                    n += 2.0;
                }
                let left_half = mx / n < 0.5 * sw as f32;
                l.weight >= 0.12 * sh as f32 && ((left_half && l.s * l.c > 0.0) || (!left_half && l.s * l.c < 0.0))
            })
            .collect(),
    };
    kept.sort_by(|a, b| b.weight.partial_cmp(&a.weight).unwrap());
    kept
}

/// Плоский список [x1,y1,x2,y2,kind,...] в координатах ИСХОДНОГО кадра.
pub fn detect(src: &[u8], w: usize, h: usize, stride: usize, thr: f32, max_segments: usize) -> Vec<i32> {
    if w < 64 || h < 64 || stride < w || src.len() < stride * (h - 1) + w {
        return Vec::new();
    }
    let (small, sw, sh, f) = downscale(src, w, h, stride);
    if sw < 32 || sh < 32 {
        return Vec::new();
    }
    let pts = ridge_points(&small, sw, sh, thr as i32);
    if pts.len() < MIN_VOTES as usize {
        return Vec::new();
    }
    let lines = select_lines(build_lines(&pts, sw, sh), sw, sh);
    let (ff, half) = (f as f32, f as f32 / 2.0);
    let mut out = Vec::new();
    'all: for l in &lines {
        for sg in &l.segs {
            if out.len() / SEG_STRIDE >= max_segments {
                break 'all;
            }
            out.extend_from_slice(&[
                (sg[0] * ff + half) as i32,
                (sg[1] * ff + half) as i32,
                (sg[2] * ff + half) as i32,
                (sg[3] * ff + half) as i32,
                if l.dashed { 0 } else { 1 },
            ]);
        }
    }
    out
}

/// C-ABI для Dart FFI. Возвращает число отрезков (по 5 i32 на отрезок) или -1 при ошибке.
///
/// # Safety
/// `data` — `len` валидных байт; `out` — `max_segments * 5` i32.
#[no_mangle]
pub unsafe extern "C" fn lane_detect(
    data: *const u8,
    len: i32,
    width: i32,
    height: i32,
    stride: i32,
    threshold: i32,
    out: *mut i32,
    max_segments: i32,
) -> i32 {
    if data.is_null() || out.is_null() || len <= 0 || width <= 0 || height <= 0 || max_segments <= 0 {
        return -1;
    }
    let src = std::slice::from_raw_parts(data, len as usize);
    let segs = detect(
        src,
        width as usize,
        height as usize,
        stride.max(width) as usize,
        threshold.clamp(10, 300) as f32,
        max_segments as usize,
    );
    let n = segs.len() / SEG_STRIDE;
    std::ptr::copy_nonoverlapping(segs.as_ptr(), out, n * SEG_STRIDE);
    n as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 720;
    const H: usize = 480;

    /// Рисует линию с перспективной шириной; dash = (рядов_штрих, рядов_пауза).
    fn draw(img: &mut [u8], stride: usize, x_at: impl Fn(f32) -> f32, dash: Option<(usize, usize)>) {
        let hy = HORIZON * H as f32;
        let y0 = (0.46 * H as f32) as usize;
        for y in y0..H - 20 {
            if let Some((on, off)) = dash {
                if (y - y0) % (on + off) >= on {
                    continue;
                }
            }
            let half = (6.0 * (y as f32 - hy) / (H as f32 - hy)).max(1.0);
            let cx = x_at(y as f32);
            let (a, b) = ((cx - half).round() as i32, (cx + half).round() as i32);
            for x in a..=b {
                if x >= 0 && (x as usize) < W {
                    img[y * stride + x as usize] = 235;
                }
            }
        }
    }
    fn left(y: f32) -> f32 { 300.0 - (y - 280.0) * 0.6 }
    fn right(y: f32) -> f32 { 420.0 + (y - 280.0) * 0.6 }

    #[test]
    fn blank_frame_gives_nothing() {
        assert!(detect(&vec![60u8; W * H], W, H, W, 55.0, 24).is_empty());
    }

    #[test]
    fn shadow_edge_and_crosswalk_are_not_markings() {
        // ступенчатая диагональная граница (тень/бордюр)
        let mut img = vec![60u8; W * H];
        for y in 0..H {
            for x in 0..W {
                if (x as f32) > 300.0 + (y as f32 - 280.0) * 0.5 {
                    img[y * W + x] = 200;
                }
            }
        }
        assert!(detect(&img, W, H, W, 55.0, 24).is_empty(), "тень принята за разметку");
        // горизонтальные полосы (зебра)
        let mut z = vec![60u8; W * H];
        for y in (300..460).step_by(40) {
            for yy in y..y + 12 {
                for x in 100..620 {
                    z[yy * W + x] = 235;
                }
            }
        }
        assert!(detect(&z, W, H, W, 55.0, 24).is_empty(), "зебра принята за разметку");
    }

    #[test]
    fn finds_dashed_left_and_solid_right_with_padding() {
        let stride = 736;
        let mut img = vec![60u8; stride * H];
        draw(&mut img, stride, left, Some((30, 30)));
        draw(&mut img, stride, right, None);
        let segs = detect(&img, W, H, stride, 55.0, 24);
        assert!(segs.len() / SEG_STRIDE >= 2, "нашли {} отрезков", segs.len() / SEG_STRIDE);
        let (mut dashed_left, mut solid_right) = (false, false);
        for s in segs.chunks(SEG_STRIDE) {
            assert!(s[1] >= (ROI_TOP * H as f32) as i32 - 4 && s[3] < H as i32);
            let mid = (s[0] + s[2]) / 2;
            if mid < 360 && s[4] == 0 { dashed_left = true }
            if mid >= 360 && s[4] == 1 { solid_right = true }
            assert!(!(mid < 360 && s[4] == 1), "левая прерывистая названа сплошной");
        }
        assert!(dashed_left, "левая прерывистая не найдена");
        assert!(solid_right, "правая сплошная не найдена");
    }

    #[test]
    fn does_not_extend_beyond_real_pixels() {
        let mut img = vec![60u8; W * H];
        let hy = HORIZON * H as f32;
        for y in 300..360 {
            let half = (6.0 * (y as f32 - hy) / (H as f32 - hy)).max(1.0);
            let cx = left(y as f32);
            for x in (cx - half) as usize..=(cx + half) as usize {
                img[y * W + x] = 235;
            }
        }
        for s in detect(&img, W, H, W, 55.0, 24).chunks(SEG_STRIDE) {
            assert!(s[1] >= 296 && s[3] <= 364, "отрезок продолжен: {:?}", s);
        }
    }

    #[test]
    fn bad_input_is_safe() {
        assert!(detect(&[0u8; 10], W, H, W, 55.0, 24).is_empty());
    }
}
