use super::*;

use cu_sensor_payloads::{CuImage, BarometerPayload, ImuPayload};
use cu_gnss_payloads::GnssFixSolution;
use cu_ahrs::AhrsPose;
use rerun::{
    ChannelDatatype, ColorModel, Image, RecordingStream, RecordingStreamBuilder, AsComponents,
};
use std::sync::atomic::{AtomicUsize, Ordering};

static IMAGE_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "firmware")]
pub type CameraSource = cu_v4l::V4l;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
pub type CameraSource = crate::sim_support::SimCameraSource;

const CAMERA_ENTITY: &str = "camera";
const AHRS_ENTITY: &str = "ahrs";
const BARO_ENTITY: &str = "baro";
const HEADING_ENTITY: &str = "heading";
const GNSS_ENTITY: &str = "gnss";
const IMU_ENTITY: &str = "imu";

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
    // Multiple inputs including image from camera
    // Use 'm lifetime
    type Input<'m> = input_msg!('m, 
        CuImage<Vec<u8>>, 
        AhrsPose,
        BarometerPayload,
        GeographicHeading,
        GnssFixSolution,
        ImuPayload
    );

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
        let (image_msg, 
            ahrs_msg, 
            baro_msg,
            heading_msg, 
            gnss_msg,
            imu_msg
        ) = *input;

        if let Some(image) = image_msg.payload() {
            self.log_image(image)?;
        }
        if let Some(ahrs) = ahrs_msg.payload() {
            self.log_ahrs(ahrs)?;
        }
        if let Some(baro) = baro_msg.payload() {
            self.log_baro(baro)?;
        }
        if let Some(heading) = heading_msg.payload() {
            self.log_heading(heading)?;
        }
        if let Some(gnss) = gnss_msg.payload() {
            self.log_gnss(gnss)?;
        }
        if let Some(imu) = imu_msg.payload() {
            self.log_imu(imu)?;
        }

        Ok(())
    }
}


// TODO: Fix all axes to default conventions in rerun, specifically rotation axes
// TODO: Fix all visualisations that use translation or rotations, e.g. pitch, yaw, etc
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
            .log(format!("{}/image", CAMERA_ENTITY), &rerun_image)
            .map_err(|e| CuError::new_with_cause("Failed to log camera", e))?;

        Ok(())
    }

    fn log_ahrs(&self, ahrs: &AhrsPose) -> CuResult<()> {
        let rq_rerun = {
            let q_avian = avian3d::math::Quaternion::from_euler(avian3d::parry::glamx::EulerRot::XZY, -ahrs.pitch.value, ahrs.yaw.value, ahrs.roll.value);
            let q_rerun = rerun::Quaternion::from_xyzw(q_avian.to_array());
            let rq_rerun = rerun::components::RotationQuat(q_rerun);
            rq_rerun
        };
        let t3d_rerun = rerun::Transform3D::new().with_quaternion(rq_rerun);
        
        self.rec
            .log(AHRS_ENTITY,  
                &[
                    &t3d_rerun as &dyn AsComponents,
                    &rerun::TransformAxes3D::new(1.0)
                    ]
                )
            .map_err(|e| CuError::new_with_cause("Failed to log ahrs", e))?;

        Ok(())
    }

    fn log_baro(&self, baro: &BarometerPayload) -> CuResult<()> {
        let pressure = rerun::Scalars::single(baro.pressure.value);
        let temperature = rerun::Scalars::single(baro.temperature.value);

        self.rec
            .log(format!("{}/pressure", BARO_ENTITY),
                &pressure
            )
            .map_err(|e| CuError::new_with_cause("Failed to log baro pressure", e))?;
        self.rec
            .log(format!("{}/temperature", BARO_ENTITY),
                &temperature
            )
            .map_err(|e| CuError::new_with_cause("Failed to log baro temperature", e))?;

        Ok(())
    }

    fn log_heading(&self, heading: &GeographicHeading) -> CuResult<()> {
        let h = rerun::Scalars::single(heading.heading.value);

        self.rec
            .log(HEADING_ENTITY,
                &h
            )
            .map_err(|e| CuError::new_with_cause("Failed to log heading", e))?;

        Ok(())
    }

    fn log_gnss(&self, gnss: &GnssFixSolution) -> CuResult<()> {
        let gp_latlon = rerun::GeoPoints::from_lat_lon([(gnss.latitude.value, gnss.longitude.value)]);
        let height_msl = rerun::Scalars::single(gnss.height_msl.value);
        
        self.rec
            .log(format!("{}/latlon", GNSS_ENTITY),
                &gp_latlon,
                )
            .map_err(|e| CuError::new_with_cause("Failed to log gnss latlon", e))?;
        self.rec
            .log(format!("{}/height_msl", GNSS_ENTITY),
                &height_msl,
                )
            .map_err(|e| CuError::new_with_cause("Failed to log gnss msl", e))?;

        Ok(())
    }

    fn log_imu(&self , imu: &ImuPayload) -> CuResult<()> {
        let accel_x = rerun::Scalars::single(imu.accel_x.value);
        let accel_y = rerun::Scalars::single(imu.accel_y.value);
        let accel_z = rerun::Scalars::single(imu.accel_z.value);
        
        let gyro_x = rerun::Scalars::single(imu.gyro_x.value);
        let gyro_y = rerun::Scalars::single(imu.gyro_y.value);
        let gyro_z = rerun::Scalars::single(imu.gyro_z.value);

        self.rec
            .log(format!("{}/accel/x", IMU_ENTITY),
                &accel_x
                )
            .map_err(|e| CuError::new_with_cause("Failed to log imu ax", e))?;
        self.rec
            .log(format!("{}/accel/y", IMU_ENTITY),
                &accel_y
                )
            .map_err(|e| CuError::new_with_cause("Failed to log imu ay", e))?;
        self.rec
            .log(format!("{}/accel/z", IMU_ENTITY),
                &accel_z
                )
            .map_err(|e| CuError::new_with_cause("Failed to log imu az", e))?;

        self.rec
            .log(format!("{}/gyro/x", IMU_ENTITY),
                &gyro_x
                )
            .map_err(|e| CuError::new_with_cause("Failed to log imu gx", e))?;
        self.rec
            .log(format!("{}/gyro/y", IMU_ENTITY),
                &gyro_y
                )
            .map_err(|e| CuError::new_with_cause("Failed to log imu gy", e))?;
        self.rec
            .log(format!("{}/gyro/z", IMU_ENTITY),
                &gyro_z
                )
            .map_err(|e| CuError::new_with_cause("Failed to log imu gz", e))?;

        Ok(())
    }

}
