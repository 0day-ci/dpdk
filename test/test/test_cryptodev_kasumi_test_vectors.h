/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TEST_CRYPTODEV_KASUMI_TEST_VECTORS_H_
#define TEST_CRYPTODEV_KASUMI_TEST_VECTORS_H_

struct kasumi_test_data {
	struct {
		uint8_t data[64];
		unsigned len;
	} key;

	struct {
		uint8_t data[64] __rte_aligned(16);
		unsigned len;
	} iv;

	/* Includes: COUNT (4 bytes) and FRESH (4 bytes) */
	struct {
		uint8_t data[8];
		unsigned len;
	} aad;

	struct {
		uint8_t data[1024]; /* Data may include direction bit */
		unsigned len; /* length must be in Bits */
	} plaintext;

	struct {
		unsigned len;
	} validDataLenInBits;

	struct {
		uint8_t data[1024];
		unsigned len; /* length must be in Bits */
	} ciphertext;

	struct {
		unsigned len;
	} validCipherLenInBits;

	/* Actual length of data to be hashed */
	struct {
		unsigned len;
	} validAuthLenInBits;

	struct {
		uint8_t data[64];
		unsigned len;
	} digest;

};

struct kasumi_test_data kasumi_test_case_1 = {
	.key = {
		.data = {
			0x2B, 0xD6, 0x45, 0x9F, 0x82, 0xC5, 0xB3, 0x00,
			0x95, 0x2C, 0x49, 0x10, 0x48, 0x81, 0xFF, 0x48
		},
		.len = 16
	},
	.iv = {
		.data = {
			0x72, 0xA4, 0xF2, 0x0F, 0x64, 0x00, 0x00, 0x00
		},
		.len = 8
	},
	.plaintext = {
		.data = {
			0x7E, 0xC6, 0x12, 0x72, 0x74, 0x3B, 0xF1, 0x61,
			0x47, 0x26, 0x44, 0x6A, 0x6C, 0x38, 0xCE, 0xD1,
			0x66, 0xF6, 0xCA, 0x76, 0xEB, 0x54, 0x30, 0x04,
			0x42, 0x86, 0x34, 0x6C, 0xEF, 0x13, 0x0F, 0x92,
			0x92, 0x2B, 0x03, 0x45, 0x0D, 0x3A, 0x99, 0x75,
			0xE5, 0xBD, 0x2E, 0xA0, 0xEB, 0x55, 0xAD, 0x8E,
			0x1B, 0x19, 0x9E, 0x3E, 0xC4, 0x31, 0x60, 0x20,
			0xE9, 0xA1, 0xB2, 0x85, 0xE7, 0x62, 0x79, 0x53,
			0x59, 0xB7, 0xBD, 0xFD, 0x39, 0xBE, 0xF4, 0xB2,
			0x48, 0x45, 0x83, 0xD5, 0xAF, 0xE0, 0x82, 0xAE,
			0xE6, 0x38, 0xBF, 0x5F, 0xD5, 0xA6, 0x06, 0x19,
			0x39, 0x01, 0xA0, 0x8F, 0x4A, 0xB4, 0x1A, 0xAB,
			0x9B, 0x13, 0x48, 0x80
		},
		.len = 800
	},
	.ciphertext = {
		.data = {
			0xD1, 0xE2, 0xDE, 0x70, 0xEE, 0xF8, 0x6C, 0x69,
			0x64, 0xFB, 0x54, 0x2B, 0xC2, 0xD4, 0x60, 0xAA,
			0xBF, 0xAA, 0x10, 0xA4, 0xA0, 0x93, 0x26, 0x2B,
			0x7D, 0x19, 0x9E, 0x70, 0x6F, 0xC2, 0xD4, 0x89,
			0x15, 0x53, 0x29, 0x69, 0x10, 0xF3, 0xA9, 0x73,
			0x01, 0x26, 0x82, 0xE4, 0x1C, 0x4E, 0x2B, 0x02,
			0xBE, 0x20, 0x17, 0xB7, 0x25, 0x3B, 0xBF, 0x93,
			0x09, 0xDE, 0x58, 0x19, 0xCB, 0x42, 0xE8, 0x19,
			0x56, 0xF4, 0xC9, 0x9B, 0xC9, 0x76, 0x5C, 0xAF,
			0x53, 0xB1, 0xD0, 0xBB, 0x82, 0x79, 0x82, 0x6A,
			0xDB, 0xBC, 0x55, 0x22, 0xE9, 0x15, 0xC1, 0x20,
			0xA6, 0x18, 0xA5, 0xA7, 0xF5, 0xE8, 0x97, 0x08,
			0x93, 0x39, 0x65, 0x0F
		},
		.len = 800
	},
	.validCipherLenInBits = {
		.len = 798
	}
};

struct kasumi_test_data kasumi_test_case_2 = {
	.key = {
		.data = {
			0xEF, 0xA8, 0xB2, 0x22, 0x9E, 0x72, 0x0C, 0x2A,
			0x7C, 0x36, 0xEA, 0x55, 0xE9, 0x60, 0x56, 0x95
		},
		.len = 16
	},
	.iv = {
		.data = {
			0xE2, 0x8B, 0xCF, 0x7B, 0xC0, 0x00, 0x00, 0x00
		},
		.len = 8
	},
	.plaintext = {
		.data = {
			0x10, 0x11, 0x12, 0x31, 0xE0, 0x60, 0x25, 0x3A,
			0x43, 0xFD, 0x3F, 0x57, 0xE3, 0x76, 0x07, 0xAB,
			0x28, 0x27, 0xB5, 0x99, 0xB6, 0xB1, 0xBB, 0xDA,
			0x37, 0xA8, 0xAB, 0xCC, 0x5A, 0x8C, 0x55, 0x0D,
			0x1B, 0xFB, 0x2F, 0x49, 0x46, 0x24, 0xFB, 0x50,
			0x36, 0x7F, 0xA3, 0x6C, 0xE3, 0xBC, 0x68, 0xF1,
			0x1C, 0xF9, 0x3B, 0x15, 0x10, 0x37, 0x6B, 0x02,
			0x13, 0x0F, 0x81, 0x2A, 0x9F, 0xA1, 0x69, 0xD8
		},
		.len = 512
	},
	.ciphertext = {
		.data = {
			0x3D, 0xEA, 0xCC, 0x7C, 0x15, 0x82, 0x1C, 0xAA,
			0x89, 0xEE, 0xCA, 0xDE, 0x9B, 0x5B, 0xD3, 0x61,
			0x4B, 0xD0, 0xC8, 0x41, 0x9D, 0x71, 0x03, 0x85,
			0xDD, 0xBE, 0x58, 0x49, 0xEF, 0x1B, 0xAC, 0x5A,
			0xE8, 0xB1, 0x4A, 0x5B, 0x0A, 0x67, 0x41, 0x52,
			0x1E, 0xB4, 0xE0, 0x0B, 0xB9, 0xEC, 0xF3, 0xE9,
			0xF7, 0xCC, 0xB9, 0xCA, 0xE7, 0x41, 0x52, 0xD7,
			0xF4, 0xE2, 0xA0, 0x34, 0xB6, 0xEA, 0x00, 0xEC
		},
		.len = 512
	},
	.validCipherLenInBits = {
		.len = 510
	}
};

struct kasumi_test_data kasumi_test_case_3 = {
	.key = {
		.data = {
			 0x5A, 0xCB, 0x1D, 0x64, 0x4C, 0x0D, 0x51, 0x20,
			 0x4E, 0xA5, 0xF1, 0x45, 0x10, 0x10, 0xD8, 0x52
		},
		.len = 16
	},
	.iv = {
		.data = {
			0xFA, 0x55, 0x6B, 0x26, 0x1C, 0x00, 0x00, 0x00
		},
		.len = 8
	},
	.aad = {
		.data = {
			0x38, 0xA6, 0xF0, 0x56, 0x05, 0xD2, 0xEC, 0x49
		},
		.len = 8
	},
	.plaintext = {
		.data = {
			0xAD, 0x9C, 0x44, 0x1F, 0x89, 0x0B, 0x38, 0xC4,
			0x57, 0xA4, 0x9D, 0x42, 0x14, 0x07, 0xE8, 0xC0
		},
		.len = 128
	},
	.ciphertext = {
		.data = {
			0x9B, 0xC9, 0x2C, 0xA8, 0x03, 0xC6, 0x7B, 0x28,
			0xA1, 0x1A, 0x4B, 0xEE, 0x5A, 0x0C, 0x25
		},
		.len = 120
	},
	.validDataLenInBits = {
			.len = 128
	},
	.validCipherLenInBits = {
		.len = 120
	},
	.validAuthLenInBits = {
		.len = 120
	},
	.digest = {
		.data = {0x87, 0x5F, 0xE4, 0x89},
		.len  = 4
	}
};

struct kasumi_test_data kasumi_test_case_4 = {
	.key = {
		.data = {
			0xD3, 0xC5, 0xD5, 0x92, 0x32, 0x7F, 0xB1, 0x1C,
			0x40, 0x35, 0xC6, 0x68, 0x0A, 0xF8, 0xC6, 0xD1
		},
		.len = 16
	},
	.iv = {
		.data = {
			0x39, 0x8A, 0x59, 0xB4, 0x2C, 0x00, 0x00, 0x00,
		},
		.len = 8
	},
	.plaintext = {
		.data = {
			0x98, 0x1B, 0xA6, 0x82, 0x4C, 0x1B, 0xFB, 0x1A,
			0xB4, 0x85, 0x47, 0x20, 0x29, 0xB7, 0x1D, 0x80,
			0x8C, 0xE3, 0x3E, 0x2C, 0xC3, 0xC0, 0xB5, 0xFC,
			0x1F, 0x3D, 0xE8, 0xA6, 0xDC, 0x66, 0xB1, 0xF0
		},
		.len = 256
	},
	.ciphertext = {
		.data = {
			0x5B, 0xB9, 0x43, 0x1B, 0xB1, 0xE9, 0x8B, 0xD1,
			0x1B, 0x93, 0xDB, 0x7C, 0x3D, 0x45, 0x13, 0x65,
			0x59, 0xBB, 0x86, 0xA2, 0x95, 0xAA, 0x20, 0x4E,
			0xCB, 0xEB, 0xF6, 0xF7, 0xA5, 0x10, 0x15, 0x10
		},
		.len = 256
	},
	.validCipherLenInBits = {
		.len = 253
	}
};

struct kasumi_test_data kasumi_test_case_5 = {
	.key = {
		.data = {
			0x60, 0x90, 0xEA, 0xE0, 0x4C, 0x83, 0x70, 0x6E,
			0xEC, 0xBF, 0x65, 0x2B, 0xE8, 0xE3, 0x65, 0x66
		},
		.len = 16
	},
	.iv = {
		.data = {
			0x72, 0xA4, 0xF2, 0x0F, 0x48, 0x00, 0x00, 0x00
		},
		.len = 8
	},
	.plaintext = {
		.data = {
			0x40, 0x98, 0x1B, 0xA6, 0x82, 0x4C, 0x1B, 0xFB,
			0x42, 0x86, 0xB2, 0x99, 0x78, 0x3D, 0xAF, 0x44,
			0x2C, 0x09, 0x9F, 0x7A, 0xB0, 0xF5, 0x8D, 0x5C,
			0x8E, 0x46, 0xB1, 0x04, 0xF0, 0x8F, 0x01, 0xB4,
			0x1A, 0xB4, 0x85, 0x47, 0x20, 0x29, 0xB7, 0x1D,
			0x36, 0xBD, 0x1A, 0x3D, 0x90, 0xDC, 0x3A, 0x41,
			0xB4, 0x6D, 0x51, 0x67, 0x2A, 0xC4, 0xC9, 0x66,
			0x3A, 0x2B, 0xE0, 0x63, 0xDA, 0x4B, 0xC8, 0xD2,
			0x80, 0x8C, 0xE3, 0x3E, 0x2C, 0xCC, 0xBF, 0xC6,
			0x34, 0xE1, 0xB2, 0x59, 0x06, 0x08, 0x76, 0xA0,
			0xFB, 0xB5, 0xA4, 0x37, 0xEB, 0xCC, 0x8D, 0x31,
			0xC1, 0x9E, 0x44, 0x54, 0x31, 0x87, 0x45, 0xE3,
			0x98, 0x76, 0x45, 0x98, 0x7A, 0x98, 0x6F, 0x2C,
			0xB0
		},
		.len = 840
	},
	.ciphertext = {
		.data = {
			0xDD, 0xB3, 0x64, 0xDD, 0x2A, 0xAE, 0xC2, 0x4D,
			0xFF, 0x29, 0x19, 0x57, 0xB7, 0x8B, 0xAD, 0x06,
			0x3A, 0xC5, 0x79, 0xCD, 0x90, 0x41, 0xBA, 0xBE,
			0x89, 0xFD, 0x19, 0x5C, 0x05, 0x78, 0xCB, 0x9F,
			0xDE, 0x42, 0x17, 0x56, 0x61, 0x78, 0xD2, 0x02,
			0x40, 0x20, 0x6D, 0x07, 0xCF, 0xA6, 0x19, 0xEC,
			0x05, 0x9F, 0x63, 0x51, 0x44, 0x59, 0xFC, 0x10,
			0xD4, 0x2D, 0xC9, 0x93, 0x4E, 0x56, 0xEB, 0xC0,
			0xCB, 0xC6, 0x0D, 0x4D, 0x2D, 0xF1, 0x74, 0x77,
			0x4C, 0xBD, 0xCD, 0x5D, 0xA4, 0xA3, 0x50, 0x31,
			0x7A, 0x7F, 0x12, 0xE1, 0x94, 0x94, 0x71, 0xF8,
			0xA2, 0x95, 0xF2, 0x72, 0xE6, 0x8F, 0xC0, 0x71,
			0x59, 0xB0, 0x7D, 0x8E, 0x2D, 0x26, 0xE4, 0x59,
			0x9E
		},
		.len = 840
	},
	.validCipherLenInBits = {
		.len = 837
	}
};

struct kasumi_test_data kasumi_test_case_6 = {
	.key = {
		.data = {
			 0x5A, 0xCB, 0x1D, 0x64, 0x4C, 0x0D, 0x51, 0x20,
			 0x4E, 0xA5, 0xF1, 0x45, 0x10, 0x10, 0xD8, 0x52
		},
		.len = 16
	},
	.iv = {
		.data = {
			0xFA, 0x55, 0x6B, 0x26, 0x1C, 0x00, 0x00, 0x00
		},
		.len = 8
	},
	.aad = {
		.data = {
			0x38, 0xA6, 0xF0, 0x56, 0x05, 0xD2, 0xEC, 0x49
		},
		.len = 8
	},
	.plaintext = {
		.data = {
			0xAD, 0x9C, 0x44, 0x1F, 0x89, 0x0B, 0x38, 0xC4,
			0x57, 0xA4, 0x9D, 0x42, 0x14, 0x07, 0xE8, 0xC0
		},
		.len = 128
	},
	.ciphertext = {
		.data = {
			0x9B, 0xC9, 0x2C, 0xA8, 0x03, 0xC6, 0x7B, 0x28,
			0xA1, 0x1A, 0x4B, 0xEE, 0x5A, 0x0C, 0x25
		},
		.len = 120
	},
	.validDataLenInBits = {
			.len = 128
	},
	.validCipherLenInBits = {
		.len = 120
	},
	.validAuthLenInBits = {
		.len = 120
	},
	.digest = {
		.data = {0x0F, 0xD2, 0xAA, 0xB5},
		.len  = 4
	}
};

#endif /* TEST_CRYPTODEV_KASUMI_TEST_VECTORS_H_ */
