use anyhow::{Context, Result};
use camino::Utf8Path;
use image::io::Reader as ImageReader;
use memmap::Mmap;
use minimp4::Mp4Muxer;
use openh264::encoder::{Encoder, EncoderConfig};
use openh264::formats::YUVBuffer;
use roslibrust::RosMessageType;
use roslibrust_util::{sensor_msgs::CompressedImage, std_msgs::Header};
use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::BufWriter;
use std::io::Cursor;
use std::io::{Read, Seek, SeekFrom};
use std::{env, fs};

fn map_mcap<P: AsRef<Utf8Path>>(p: P) -> Result<Mmap> {
    let fd = fs::File::open(p.as_ref()).context("Couldn't open MCAP file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

// TODO(lucasw) make a mcap_tools utility function for this
fn mcap_write<T: RosMessageType>(
    mcap_out: &mut mcap::Writer<BufWriter<File>>,
    msg: &T,
    header: Header,
    channel_id: u16,
) -> Result<(), mcap::McapError> {
    let sequence = 0;
    let data = serde_rosmsg::to_vec(&msg).unwrap();
    let log_time = tf_roslibrust::tf_util::stamp_to_duration(&header.stamp)
        .num_nanoseconds()
        .unwrap() as u64;
    mcap_out.write_to_known_channel(
        &mcap::records::MessageHeader {
            channel_id,
            sequence,
            log_time,
            publish_time: log_time,
        },
        &data[4..], // chop off header bytes
    )
}

fn read_it(input_mcap_name: &str) -> Result<()> {
    let mapped = map_mcap(input_mcap_name)?;

    let mut encoders_by_topic: HashMap<String, Encoder> = HashMap::new();

    // Map of topic -> channel for the topic
    let mut topic_channel_ids: HashMap<String, u16> = HashMap::new();

    // TODO(lucasw) disable zstd compression, it isn't going to do anything
    let mut video_mcap = mcap::Writer::new(BufWriter::new(
        File::create("compressed_video.mcap").unwrap(),
    ))
    .unwrap();

    // TODO(lucasw) make writing to these optional
    // these files can be played with mplayer or vlc but don't have the correct frame rate
    // and generate error messages
    let mut h264_data_by_topic: HashMap<String, (usize, usize, Vec<u8>)> = HashMap::new();

    let schema_id = video_mcap.add_schema(
        CompressedImage::ROS_TYPE_NAME,
        "ros1msg",
        CompressedImage::DEFINITION.as_bytes(),
    )?;

    let mut connection_summary = BTreeMap::new();

    let mut count = 0;
    for message in mcap::MessageStream::new(&mapped)? {
        let full_message = message.unwrap();
        let schema = full_message.channel.schema.as_ref().unwrap().clone();
        if schema.name.ne("sensor_msgs/CompressedImage") {
            continue;
        }

        count += 1;
        println!("{count}");

        let msg_with_header = roslibrust_util::get_message_data_with_header(full_message.data);
        let msg = serde_rosmsg::from_slice::<CompressedImage>(&msg_with_header).unwrap();

        // println!("{:?}", msg);
        let timestamp = msg.header.stamp;
        let frame_id = msg.header.frame_id;
        let data = msg.data;

        let reader = ImageReader::new(Cursor::new(data))
            .with_guessed_format()
            .expect("Cursor io never fails");

        let img = reader.decode()?;

        let rgb8 = &img.to_rgb8();

        let width = usize::try_from(rgb8.width()).unwrap();
        let height = usize::try_from(rgb8.height()).unwrap();

        let topic = std::format!("{topic}_video", topic = full_message.channel.topic);

        let (width2, height2, h264_data) =
            h264_data_by_topic.entry(topic.clone()).or_insert_with(|| {
                println!("{topic}");
                (width, height, Vec::new())
            });
        assert!(width == *width2);
        assert!(height == *height2);

        let encoder = encoders_by_topic.entry(topic.clone()).or_insert_with(|| {
            // fixme - command line argument for bitrate
            let config =
                EncoderConfig::new(rgb8.width(), rgb8.height()).set_bitrate_bps(10_000_000);
            Encoder::with_config(config).unwrap()
        });

        let yuv = YUVBuffer::with_rgb(width, height, rgb8);
        let bitstream = encoder.encode(&yuv).unwrap();
        bitstream.write_vec(h264_data);

        let mut out_msg = CompressedImage::default();
        out_msg.header.stamp = timestamp;
        out_msg.header.frame_id = frame_id;
        out_msg.format = "h264".to_string();
        out_msg.data = bitstream.to_vec();

        let channel_id = topic_channel_ids
            .entry(topic.clone())
            .or_insert_with_key(|topic| {
                connection_summary.insert("topic".to_string(), topic.to_string());
                video_mcap
                    .add_channel(schema_id, topic, "ros1", &connection_summary)
                    .unwrap()
            });

        mcap_write::<CompressedImage>(
            &mut video_mcap,
            &out_msg,
            out_msg.header.clone(),
            *channel_id,
        )?;
    }

    for (topic, (width, height, h264_data)) in h264_data_by_topic {
        let name = format!("{}.mp4", topic.replace("/", "_"));
        println!("muxing h264 to {name}");
        let mut video_buffer = Cursor::new(Vec::new());
        let mut mp4muxer = Mp4Muxer::new(&mut video_buffer);
        mp4muxer.init_video(width as i32, height as i32, false, &topic);
        mp4muxer.write_video(&h264_data);
        mp4muxer.close();
        video_buffer.seek(SeekFrom::Start(0))?;
        let mut video_bytes = Vec::new();
        video_buffer.read_to_end(&mut video_bytes)?;
        std::fs::write(&name, &video_bytes)?;
    }

    video_mcap.finish().unwrap();
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    let input_mcap_name = &args[1];
    read_it(input_mcap_name)?;
    Ok(())
}
