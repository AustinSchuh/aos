//! Turning a camera's gray image into what the encoder takes.

/// The size the encoder is given for a `cols` x `rows` camera image.  I420 needs even sizes, and
/// `half` halves each side first.
pub fn stream_size(cols: u32, rows: u32, half: bool) -> (u32, u32) {
    let (width, height) = if half {
        (cols / 2, rows / 2)
    } else {
        (cols, rows)
    };
    (width & !1, height & !1)
}

/// Writes `gray` (`cols` x `rows`, one byte a pixel, rows packed) into `out` as an I420 frame of
/// [`stream_size()`]: the gray as the Y plane, either as is or averaged over 2x2 blocks when
/// `half`, and neutral chroma.  Returns the frame's size.
///
/// # Panics
///
/// If `gray` is shorter than `cols * rows`.
pub fn gray_to_i420(
    gray: &[u8],
    cols: u32,
    rows: u32,
    half: bool,
    out: &mut Vec<u8>,
) -> (u32, u32) {
    let (cols, rows) = (cols as usize, rows as usize);
    assert!(
        gray.len() >= cols * rows,
        "{} bytes for a {cols}x{rows} image",
        gray.len()
    );
    let (width, height) = stream_size(cols as u32, rows as u32, half);
    let (w, h) = (width as usize, height as usize);
    let luma = w * h;
    out.clear();
    out.resize(luma + 2 * (w / 2) * (h / 2), 128);

    let y = &mut out[..luma];
    if half {
        for (out_row, y_row) in y.chunks_exact_mut(w).enumerate() {
            let top = &gray[2 * out_row * cols..][..2 * w];
            let bottom = &gray[(2 * out_row + 1) * cols..][..2 * w];
            for (x, pixel) in y_row.iter_mut().enumerate() {
                let sum = top[2 * x] as u16
                    + top[2 * x + 1] as u16
                    + bottom[2 * x] as u16
                    + bottom[2 * x + 1] as u16;
                // Round to nearest.
                *pixel = ((sum + 2) / 4) as u8;
            }
        }
    } else {
        for (row, y_row) in y.chunks_exact_mut(w).enumerate() {
            y_row.copy_from_slice(&gray[row * cols..][..w]);
        }
    }
    (width, height)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_size_copies_luma_and_greys_chroma() {
        let gray: Vec<u8> = (0..4 * 2).map(|i| i as u8 * 10).collect();
        let mut out = Vec::new();
        assert_eq!(gray_to_i420(&gray, 4, 2, false, &mut out), (4, 2));
        assert_eq!(&out[..8], &gray[..]);
        // 2x1 of U, then 2x1 of V.
        assert_eq!(&out[8..], &[128, 128, 128, 128]);
    }

    #[test]
    fn half_size_averages_each_block() {
        #[rustfmt::skip]
        let gray = [
            0, 4, 100, 100,
            8, 4, 200, 200,
        ];
        let mut out = Vec::new();
        assert_eq!(gray_to_i420(&gray, 4, 2, true, &mut out), (2, 0));
        // 2x1 is odd in height, so it rounds down to nothing.
        assert!(out.is_empty());

        #[rustfmt::skip]
        let gray = [
            0, 4,   100, 100,  1, 1,  9, 9,
            8, 4,   200, 200,  1, 2,  9, 9,
            10, 10, 20, 20,    3, 3,  0, 255,
            10, 10, 20, 21,    3, 3,  0, 255,
        ];
        assert_eq!(gray_to_i420(&gray, 8, 4, true, &mut out), (4, 2));
        // (0+4+8+4)/4 = 4, (100+100+200+200)/4 = 150, (1+1+1+2)/4 = 1.25 -> 1, 9; then 10,
        // (20+20+20+21)/4 = 20.25 -> 20, 3, (0+255+0+255)/4 = 127.5 -> 128.
        assert_eq!(&out[..8], &[4, 150, 1, 9, 10, 20, 3, 128]);
        assert_eq!(&out[8..], &[128; 4]);
    }

    #[test]
    fn odd_sizes_drop_the_last_row_and_column() {
        let gray: Vec<u8> = (0..5 * 3).map(|i| i as u8).collect();
        let mut out = Vec::new();
        assert_eq!(gray_to_i420(&gray, 5, 3, false, &mut out), (4, 2));
        assert_eq!(&out[..8], &[0, 1, 2, 3, 5, 6, 7, 8]);
        assert_eq!(stream_size(1600, 1304, true), (800, 652));
        assert_eq!(stream_size(1280, 800, false), (1280, 800));
    }
}
