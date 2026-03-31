use super::*;

use cu_sensor_payloads::CuImage;
use cu29::prelude::*;
use rerun::{
    Boxes2D, ChannelDatatype, ColorModel, Image, LineStrips2D, Points2D, RecordingStream,
    RecordingStreamBuilder, Vec2D,
};
use std::sync::atomic::{AtomicUsize, Ordering};

static IMAGE_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "firmware")]
pub type CameraSource = cu_v4l::V4l;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
pub type CameraSource = crate::sim_support::SimCameraSource;

const CAMERA_ENTITY: &str = "camera/image";

/// Convert RGB to RGBA.
pub fn rgb_to_rgba(data: &[u8], _width: usize, _height: usize) -> Vec<u8> {
    data.chunks(3)
        .flat_map(|rgb| {
            if rgb.len() >= 3 {
                [rgb[0], rgb[1], rgb[2], 255]
            } else {
                [0, 0, 0, 255]
            }
        })
        .collect()
}


#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct RerunViz {
    #[reflect(ignore)]
    rec: RecordingStream,
}

impl Freezable for RerunViz {}

impl CuSinkTask for RerunViz {
    type Resources<'r> = ();
    // One input: image from camera
    // Use 'm lifetime
    type Input<'m> = input_msg!('m, CuImage<Vec<u8>>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let rec = RecordingStreamBuilder::new("Flight Controller")
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn Rerun stream", e))?;

        Ok(Self { rec })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        // Input is a reference to a tuple of message references
        let image_msg = input;

        // Log the camera image if available
        if let Some(image) = image_msg.payload() {
            self.log_image(image)?;
        }

        Ok(())
    }
}

impl RerunViz {
    fn log_image(&self, image: &CuImage<Vec<u8>>) -> CuResult<()> {
        let width = image.format.width;
        let height = image.format.height;
        let pixel_format = &image.format.pixel_format;

        // Convert image to RGBA for visualization
        let rgba_data: Vec<u8> = image.buffer_handle.with_inner(|data| {
            match pixel_format {
                b"RGBA" => data.to_vec(),
                b"RGB " => rgb_to_rgba(data, width as usize, height as usize),
                _ => {data.to_vec()}
                }
            }
        );

        let log_idx = IMAGE_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
        if log_idx < 5 {
            info!(
                "rerun_viz: width={} height={} pixel_format={} rgba_len={}",
                width,
                height,
                String::from_utf8_lossy(pixel_format),
                rgba_data.len()
            );
        }

        let rerun_image = Image::from_color_model_and_bytes(
            rgba_data,
            [width, height],
            ColorModel::RGBA,
            ChannelDatatype::U8,
        );

        self.rec
            .log(CAMERA_ENTITY, &rerun_image)
            .map_err(|e| CuError::new_with_cause("Failed to log image", e))?;

        Ok(())
    }

}
