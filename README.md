# mcap-videoify

Convert ros1 sensor_msgs/CompressedImage messages (jpg or anything ImageReader with_guessed_format can handle) in an mcap to h264 also using the CompresedImage container.

```
cargo run --release --bin mcap_convert_compressed_to_h264 /path/to/input_file.mcap
```

This outputs compressed_video.mcap with only compressed image topics found in the input mcap, but with topics renamed to have \_video at the end.

Also decode live with roslibrust:

```
cargo run --release --bin h264_to_image compressed_video:=/my/image/compressed_video
```

and then rqt_image_view can view them using `mcap_play compressed_video.mcap` (mcap_play from http://github.com/lucasw/mcap_tools/).

TODO:
* convert regular Images as well as CompressedImage
* convert from h264 to jpg in mcaps
* live encoder node for Image or CompressedImage
* live decoder to jpg compressed
* player node that displays in minifb
