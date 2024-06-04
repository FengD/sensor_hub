#include "tools/service_server/data_encode_decode.h"

namespace crdc {
namespace airi {
const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
std::string DataEncoderDecoder::base64_encode(const unsigned char* data, size_t len) {
  std::string encoded;
  encoded.reserve(((len + 2) / 3) * 4);

  for (size_t i = 0; i < len; i += 3) {
    unsigned int a = data[i];
    unsigned int b = (i + 1 < len) ? data[i + 1] : 0;
    unsigned int c = (i + 2 < len) ? data[i + 2] : 0;
    unsigned int combined = (a << 16) | (b << 8) | c;

    encoded.push_back(base64_chars[(combined >> 18) & 0x3F]);
    encoded.push_back(base64_chars[(combined >> 12) & 0x3F]);

    if (i + 1 < len) {
      encoded.push_back(base64_chars[(combined >> 6) & 0x3F]);
    }
    if (i + 2 < len) {
      encoded.push_back(base64_chars[combined & 0x3F]);
    }
  }

  while (encoded.size() % 4 != 0) {
    encoded.push_back('=');
  }
  return encoded;
}

std::string DataEncoderDecoder::base64_decode(const std::string& encoded_data) {
  std::string decoded_data;
  size_t len = encoded_data.size();
  size_t padding = 0;

  if (len > 0 && encoded_data[len - 1] == '=') {
    padding++;
    if (len > 1 && encoded_data[len - 2] == '=') {
      padding++;
    }
  }

  size_t decoded_len = (len * 3) / 4 - padding;
  decoded_data.reserve(decoded_len);

  for (size_t i = 0; i < len; i += 4) {
    unsigned int a = std::find(base64_chars, base64_chars + 64, encoded_data[i]) - base64_chars;
    unsigned int b = std::find(base64_chars, base64_chars + 64, encoded_data[i + 1]) - base64_chars;
    unsigned int c = std::find(base64_chars, base64_chars + 64, encoded_data[i + 2]) - base64_chars;
    unsigned int d = std::find(base64_chars, base64_chars + 64, encoded_data[i + 3]) - base64_chars;
    unsigned int combined = (a << 18) | (b << 12) | (c << 6) | d;

    decoded_data.push_back((combined >> 16) & 0xFF);
    if (i + 2 < len - padding) {
      decoded_data.push_back((combined >> 8) & 0xFF);
    }
    if (i + 3 < len - padding) {
      decoded_data.push_back(combined & 0xFF);
    }
  }

  return decoded_data;
}

std::string DataEncoderDecoder::convert_image_to_base64(cv::Mat &image) {
  std::vector<uint8_t> buffer;
  cv::imencode(".png", image, buffer);
  std::string base64_image = base64_encode(buffer.data(), buffer.size());
  return base64_image;
}
}  // namespace airi
}  // namespace crdc
