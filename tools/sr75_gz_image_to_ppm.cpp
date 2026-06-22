#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

namespace
{
struct Config
{
  std::string topic{"/sr75/chase_camera/image"};
  std::string output{"/tmp/sr75_chase_camera.ppm"};
  double rateHz{30.0};
};

class PpmWriter
{
  public: explicit PpmWriter(Config _config)
      : config(std::move(_config))
  {
    this->minInterval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / this->config.rateHz));
  }

  public: void OnImage(const gz::msgs::Image &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    const auto now = std::chrono::steady_clock::now();
    if (now - this->lastWrite < this->minInterval)
      return;

    const auto width = _msg.width();
    const auto height = _msg.height();
    if (width == 0 || height == 0)
    {
      std::cerr << "Received empty image: " << width << "x" << height << std::endl;
      return;
    }

    const auto format = _msg.pixel_format_type();
    std::string rgbData;
    if (!this->ConvertToRgb(_msg, format, rgbData))
    {
      if (format != this->lastUnsupportedFormat)
      {
        std::cerr << "Unsupported pixel format: " << static_cast<int>(format) << std::endl;
        this->lastUnsupportedFormat = format;
      }
      return;
    }

    if (!this->WritePpmAtomic(width, height, rgbData))
      return;

    this->lastWrite = now;
  }

  private: bool ConvertToRgb(
      const gz::msgs::Image &_msg,
      int _format,
      std::string &_rgbData) const
  {
    const std::string &data = _msg.data();
    switch (_format)
    {
      case gz::msgs::PixelFormatType::RGB_INT8:
      {
        const size_t expected = static_cast<size_t>(_msg.width()) * _msg.height() * 3u;
        if (data.size() < expected)
        {
          std::cerr << "RGB_INT8 payload too small: got " << data.size()
                    << ", expected at least " << expected << std::endl;
          return false;
        }
        _rgbData.assign(data.data(), expected);
        return true;
      }
      case gz::msgs::PixelFormatType::BGR_INT8:
      {
        const size_t pixelCount = static_cast<size_t>(_msg.width()) * _msg.height();
        const size_t expected = pixelCount * 3u;
        if (data.size() < expected)
        {
          std::cerr << "BGR_INT8 payload too small: got " << data.size()
                    << ", expected at least " << expected << std::endl;
          return false;
        }
        _rgbData.resize(expected);
        for (size_t i = 0; i < pixelCount; ++i)
        {
          const size_t src = i * 3u;
          _rgbData[src + 0] = data[src + 2];
          _rgbData[src + 1] = data[src + 1];
          _rgbData[src + 2] = data[src + 0];
        }
        return true;
      }
      case gz::msgs::PixelFormatType::RGBA_INT8:
      case gz::msgs::PixelFormatType::BGRA_INT8:
      {
        const bool bgra = (_format == gz::msgs::PixelFormatType::BGRA_INT8);
        const size_t pixelCount = static_cast<size_t>(_msg.width()) * _msg.height();
        const size_t expected = pixelCount * 4u;
        if (data.size() < expected)
        {
          std::cerr << "RGBA/BGRA payload too small: got " << data.size()
                    << ", expected at least " << expected << std::endl;
          return false;
        }
        _rgbData.resize(pixelCount * 3u);
        for (size_t i = 0; i < pixelCount; ++i)
        {
          const size_t src = i * 4u;
          const size_t dst = i * 3u;
          if (bgra)
          {
            _rgbData[dst + 0] = data[src + 2];
            _rgbData[dst + 1] = data[src + 1];
            _rgbData[dst + 2] = data[src + 0];
          }
          else
          {
            _rgbData[dst + 0] = data[src + 0];
            _rgbData[dst + 1] = data[src + 1];
            _rgbData[dst + 2] = data[src + 2];
          }
        }
        return true;
      }
      default:
        return false;
    }
  }

  private: bool WritePpmAtomic(
      unsigned int _width,
      unsigned int _height,
      const std::string &_rgbData) const
  {
    const std::string tmpPath = this->config.output + ".tmp";
    std::ofstream out(tmpPath, std::ios::binary | std::ios::trunc);
    if (!out.is_open())
    {
      std::cerr << "Failed to open " << tmpPath << " for writing" << std::endl;
      return false;
    }

    out << "P6\n" << _width << " " << _height << "\n255\n";
    out.write(_rgbData.data(), static_cast<std::streamsize>(_rgbData.size()));
    out.close();

    if (!out)
    {
      std::cerr << "Failed to write image data to " << tmpPath << std::endl;
      return false;
    }

    if (std::rename(tmpPath.c_str(), this->config.output.c_str()) != 0)
    {
      std::cerr << "Failed to rename " << tmpPath << " to "
                << this->config.output << ": " << std::strerror(errno) << std::endl;
      return false;
    }

    return true;
  }

  private: Config config;
  private: std::chrono::steady_clock::duration minInterval;
  private: std::chrono::steady_clock::time_point lastWrite{};
  private: std::mutex mutex;
  private: int lastUnsupportedFormat{-1};
};

Config ParseArgs(int _argc, char **_argv)
{
  Config config;
  for (int i = 1; i < _argc; ++i)
  {
    const std::string arg = _argv[i];
    auto requireValue = [&](const char *_name) -> const char * {
      if (i + 1 >= _argc)
      {
        std::cerr << "Missing value for " << _name << std::endl;
        std::exit(2);
      }
      return _argv[++i];
    };

    if (arg == "--topic")
      config.topic = requireValue("--topic");
    else if (arg == "--output")
      config.output = requireValue("--output");
    else if (arg == "--rate")
      config.rateHz = std::stod(requireValue("--rate"));
    else if (arg == "--help" || arg == "-h")
    {
      std::cout
          << "Usage: sr75_gz_image_to_ppm [--topic TOPIC] [--output FILE] [--rate HZ]\n"
          << "Defaults:\n"
          << "  --topic  /sr75/chase_camera/image\n"
          << "  --output /tmp/sr75_chase_camera.ppm\n"
          << "  --rate   30\n";
      std::exit(0);
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << std::endl;
      std::exit(2);
    }
  }

  if (config.rateHz <= 0.0)
  {
    std::cerr << "--rate must be greater than zero" << std::endl;
    std::exit(2);
  }
  return config;
}
}  // namespace

int main(int argc, char **argv)
{
  const Config config = ParseArgs(argc, argv);
  std::cout << "SR-75 Gazebo image to PPM helper\n"
            << "  Topic:  " << config.topic << "\n"
            << "  Output: " << config.output << "\n"
            << "  Rate:   " << config.rateHz << " Hz" << std::endl;

  gz::transport::Node node;
  auto writer = std::make_shared<PpmWriter>(config);
  std::function<void(const gz::msgs::Image &)> callback =
      [writer](const gz::msgs::Image &_msg)
      {
        writer->OnImage(_msg);
      };

  if (!node.Subscribe<gz::msgs::Image>(config.topic, callback))
  {
    std::cerr << "Failed to subscribe to " << config.topic << std::endl;
    return 1;
  }

  std::cout << "Subscribed. Writing latest frames to " << config.output << std::endl;
  while (true)
    std::this_thread::sleep_for(std::chrono::seconds(1));

  return 0;
}
