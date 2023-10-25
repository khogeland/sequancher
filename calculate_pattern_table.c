#include <stdint.h>
#include <stdio.h>

const uint8_t flips[40][2] = {{6, 10}, {4, 8}, {7, 11}, {5, 9}, {7, 8}, {3, 12}, {4, 10}, {0, 5}, {5, 14}, {0, 11}, {3, 7}, {1, 7}, {6, 3}, {5, 2}, {2, 9}, {7, 8}, {0, 15}, {1, 14}, {2, 13}, {3, 12}, {4, 11}, {5, 10}, {6, 9}, {7, 8}, {8, 7}, {13, 6}, {10, 13}, {9, 12}, {14, 8}, {12, 8}, {15, 4}, {10, 1}, {15, 10}, {11, 5}, {12, 3}, {8, 7}, {10, 6}, {8, 4}, {11, 7}, {9, 5}};
const uint8_t num_flips = sizeof(flips) / sizeof(flips[0]);
const uint8_t p_size = num_flips+1;
const uint8_t f_size = 17;

uint16_t flip(uint16_t pattern, uint8_t step) {
  uint16_t res = pattern;
  for (uint8_t i = 0; i < step; i++) {
    uint8_t f1 = flips[i][0];
    uint8_t f2 = flips[i][1];
    uint8_t a = (pattern >> f1) & 1;
    uint8_t b = (pattern >> f2) & 1;
    uint16_t x = a ^ b;
    x = (x << f1) | (x << f2);
    pattern = pattern ^ x;
  }
  return pattern;
}

uint16_t fades[17] = {
  0,
  0b1,
  0b11,
  0b111,
  0b1111,
  0b11111,
  0b111111,
  0b1111111,
  0b11111111,
  0b111111111,
  0b1111111111,
  0b11111111111,
  0b111111111111,
  0b1111111111111,
  0b11111111111111,
  0b111111111111111,
  0b1111111111111111,
};

int main() {
  printf("#include <stdint.h>\n");
  printf("const uint8_t num_patterns = %d;\n", p_size);
  printf("const uint8_t num_fades = %d;\n", f_size);
  printf("const uint16_t pattern_table[%d][%d] = {", p_size, f_size);
  for (uint8_t p = 0; p < p_size; p++) { 
    printf("\n\t{");
    for (uint8_t f = 0; f < f_size; f++) {
      printf("{%d},", flip(fades[f], p));
    }
    printf("},");
  }
  printf("\n};\n");
}
