// LsuUtils.h
#include <cstdint>

class LsuUtils {
public:
  // 解析 func3 得到字节大小 (用于 Store 或者 Debug)
  static int get_size_in_bytes(int func3) {
    // func3 & 0x3 得到 0,1,2,3
    // 对应 1, 2, 4, 8 字节
    return 1 << (func3 & 0b11);
  }

  // 通用的数据处理函数
  // 输入：raw_data (从 Cache 读回来的原始 64位/32位 数据)
  // 输出：写入寄存器的最终值 (已移位、掩码、符号扩展)
  static uint64_t align_and_sign_extend(uint64_t raw_data, uint64_t addr,
                                        int func3) {
    int offset_bits = (addr & 0b111) * 8; // 计算位偏移
    int size_code = func3 & 0b11;         // 0=B, 1=H, 2=W, 3=D
    bool is_unsigned = (func3 & 0b100);   // Bit 2 是 unsigned 标志

    // 1. 移位 (Alignment)
    // 将目标数据移到最低位
    uint64_t shifted_data = raw_data >> offset_bits;

    // 2. 掩码生成 (Masking)
    // 技巧：利用 size_code 生成掩码
    // size=0(1B) -> mask=0xFF
    // size=1(2B) -> mask=0xFFFF
    // size=2(4B) -> mask=0xFFFFFFFF
    // size=3(8B) -> mask=0xFF..FF
    uint64_t mask = (uint64_t)-1;
    if (size_code < 3) {
      mask = (1ULL << (8 * (1 << size_code))) - 1;
    }

    uint64_t val = shifted_data & mask;

    // 3. 符号扩展 (Sign Extension)
    // 只有 Load 且不是 Unsigned 时才做
    if (!is_unsigned && size_code < 3) {
      // 检查符号位
      uint64_t sign_bit_pos = (8 * (1 << size_code)) - 1;
      if ((val >> sign_bit_pos) & 1) {
        // 如果符号位是1，把高位全部置1
        // 技巧：利用按位取反的掩码
        val |= ~mask;
      }
    }

    return val;
  }

  static int get_mem_width(int func3) {
    switch (func3 & 0b11) {
    case 0b00:
      return 1;
    case 0b01:
      return 2;
    case 0b10:
      return 4;
    default:
      return 4;
    }
  }

  static uint32_t extract_data(uint32_t raw_mem_val, uint32_t addr, int func3) {
    const int bit_offset = (addr & 0x3) * 8;
    uint32_t result = 0;
    const uint32_t shifted = raw_mem_val >> bit_offset;

    switch (func3) {
    case 0b000:
      result = shifted & 0xFF;
      if (result & 0x80) {
        result |= 0xFFFFFF00;
      }
      break;
    case 0b001:
      result = shifted & 0xFFFF;
      if (result & 0x8000) {
        result |= 0xFFFF0000;
      }
      break;
    case 0b010:
      result = shifted;
      break;
    case 0b100:
      result = shifted & 0xFF;
      break;
    case 0b101:
      result = shifted & 0xFFFF;
      break;
    default:
      result = shifted;
      break;
    }
    return result;
  }

  static uint32_t merge_data_to_word(uint32_t old_word, uint32_t new_data,
                                     uint32_t addr, int func3) {
    const int bit_offset = (addr & 0x3) * 8;
    uint32_t mask = 0;

    switch (func3 & 0b11) {
    case 0b00:
      mask = 0xFF;
      break;
    case 0b01:
      mask = 0xFFFF;
      break;
    default:
      mask = 0xFFFFFFFF;
      break;
    }

    const uint32_t clear_mask = ~(mask << bit_offset);
    uint32_t result = old_word & clear_mask;
    result |= ((new_data & mask) << bit_offset);
    return result;
  }

  static inline uint8_t get_store_strb(uint32_t addr, uint8_t func3) {
    uint32_t off = addr & 0x3;
    switch (func3 & 0x3) {
    case 0:
      return 0x1u << off; // SB
    case 1:
      return 0x3u << off; // SH, 要求 off 为 0 或 2
    case 2:
      return 0xFu; // SW, 要求 off 为 0
    default:
      return 0;
    }
  }

  static inline uint32_t align_store_data(uint32_t data, uint32_t addr,
                                          uint8_t func3) {
    uint32_t off = addr & 0x3;
    switch (func3 & 0x3) {
    case 0:
      return (data & 0x000000FFu) << (off * 8);
    case 1:
      return (data & 0x0000FFFFu) << (off * 8);
    case 2:
      return data;
    default:
      return 0;
    }
  }
};
