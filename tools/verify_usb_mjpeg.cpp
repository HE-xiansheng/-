#include <usb_cam/usb_cam.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<char> jpeg((std::istreambuf_iterator<char>(input)), {});
  cv::Mat encoded(1, static_cast<int>(jpeg.size()), CV_8U, jpeg.data());
  cv::Mat ref = cv::imdecode(encoded, cv::IMREAD_COLOR);
  if (ref.cols != 1280 || ref.rows != 720) return 3;
  cv::cvtColor(ref, ref, cv::COLOR_BGR2RGB);
  usb_cam::formats::format_arguments_t args;
  args.width = ref.cols; args.height = ref.rows;
  args.pixels = ref.cols * ref.rows; args.av_device_format_str = "YUV422P";
  usb_cam::formats::MJPEG2RGB decoder(args);
  if (decoder.ros() != "rgb8" || decoder.channels() != 3 || !decoder.requires_conversion()) return 4;
  const auto size = ref.total() * 3;
  std::vector<char> output(size + 128, 0x5a);
  const auto start = std::chrono::steady_clock::now();
  double error = 0;
  for (int frame = 0; frame < 5; ++frame) {
    const char *src = jpeg.data(); char *dest = output.data() + 64;
    decoder.convert(src, dest, static_cast<int>(jpeg.size()));
    cv::Mat decoded(ref.rows, ref.cols, CV_8UC3, dest), diff;
    cv::absdiff(ref, decoded, diff);
    auto means = cv::mean(diff); error = (means[0] + means[1] + means[2]) / 3;
    if (error > 20) return 5;
    for (size_t i = 0; i < 64; ++i) {
      if (output[i] != 0x5a || output[size+64+i] != 0x5a) return 6;
    }
  }
  const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count()/5;
  std::cout << "{\"passed\":true,\"frames\":5,\"width\":1280,\"height\":720,\"encoding\":\"rgb8\",\"mean_abs_error\":" << error
            << ",\"decode_ms\":" << ms << ",\"buffer_guards_intact\":true}" << std::endl;
}
