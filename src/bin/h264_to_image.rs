/*!
Lucas Walter

Decode and republish an h264 stream within a ros1 CompressedImage message
*/

use openh264::decoder::Decoder;
use roslibrust::ros1::NodeHandle;
use roslibrust_util::sensor_msgs::{CompressedImage, Image};
use std::collections::HashMap;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    let (nh, _full_node_name, _params, remaps, _remaining_args) = {
        let mut params = HashMap::from([("_name".to_string(), "h264_to_image".to_string())]);
        let mut remaps = HashMap::from([
            (
                "compressed_video".to_string(),
                "compressed_video".to_string(),
            ),
            ("image".to_string(), "image".to_string()),
        ]);

        let (_ns, full_node_name, remaining_args) =
            roslibrust_util::get_params_remaps(&mut params, &mut remaps);

        let ros_master_uri =
            std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());
        let nh = NodeHandle::new(&ros_master_uri, &full_node_name).await?;
        println!("{full_node_name} connected to roscore at {ros_master_uri}");

        (nh, full_node_name, params, remaps, remaining_args)
    };

    {
        let mut decoder = Decoder::new()?;

        let h264_topic = remaps.get("compressed_video").unwrap().clone();
        let mut h264_subscriber = nh.subscribe::<CompressedImage>(&h264_topic, 100).await?;
        println!("{h264_topic}");

        let image_topic = remaps.get("image").unwrap().clone();
        let image_publisher = nh.advertise::<Image>(&image_topic, 10, false).await?;
        println!("{image_topic}");

        loop {
            let h264_msg = h264_subscriber.next().await.unwrap().unwrap();

            let yuv = decoder.decode(&h264_msg.data);
            if let Ok(Some(yuv)) = yuv {
                // DecodedYUV { info: TagSysMemBuffer { iWidth: 1936, iHeight: 1464, iFormat: 23, iStride: [2016, 1008] }, y: [...
                // println!("{yuv:?}");
                let (width, height) = yuv.dimension_rgb();
                let size = width * height * 3;
                let mut rgb_out = vec![0; size];

                yuv.write_rgb8(&mut rgb_out);

                // println!("{rgb_out:?}");
                let mut image_msg = Image::default();
                image_msg.header = h264_msg.header;
                image_msg.width = width as u32;
                image_msg.height = height as u32;
                image_msg.step = image_msg.width * 3;
                image_msg.is_bigendian = 0;
                image_msg.encoding = "rgb8".to_string();
                image_msg.data = rgb_out;
                image_publisher.publish(&image_msg).await?;
            } else {
                println!("decode {decoder:?}");
            }
        }
    }

    // Ok(())
}
