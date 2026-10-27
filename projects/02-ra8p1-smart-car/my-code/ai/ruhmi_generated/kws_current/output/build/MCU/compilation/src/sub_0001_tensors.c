#include "sub_0001_tensors.h"

const TensorInfo sub_0001_tensors[] = {
  { "_split_1_command_stream", 1, 3232, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 2, 189200, "MODEL", 0xffffffff },
  { "_split_1_scratch", 3, 156800, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 4, 156800, "FAST_SCRATCH", 0x0 },
  { "serving_default_input_2_0_10032", 5, 3920, "INPUT_TENSOR", 0x1ea00 },
  { "StatefulPartitionedCall_0_70023_10076", 0, 14, "OUTPUT_TENSOR", 0x0 },
};

const size_t sub_0001_tensors_count = sizeof(sub_0001_tensors) / sizeof(sub_0001_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0001_address_serving_default_input_2_0_10032 = 0x1ea00;
const uint32_t sub_0001_address_StatefulPartitionedCall_0_70023_10076 = 0x0;

