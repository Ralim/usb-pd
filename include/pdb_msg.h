/*
 * PD Buddy Firmware Library - USB Power Delivery for everyone
 * Copyright 2017-2018 Clayton G. Hobbs
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PDB_MSG_H
#define PDB_MSG_H

#include <stdint.h>

/*
 * PD message union
 *
 * This can be safely read from or written to in any form without any
 * transformations because everything in the system is little-endian.
 *
 * Two bytes of padding are required at the start to prevent problems due to
 * alignment.  Specifically, without the padding, &obj[0] != &bytes[2], making
 * the statement in the previous paragraph invalid.
 */
typedef union {
	struct {
		uint8_t _pad1[2];
		uint8_t bytes[30];
	} __attribute__((packed));
	struct {
		uint8_t _pad2[2];
		uint16_t hdr;
		union {
			uint32_t obj[7];
			struct {
				uint16_t exthdr;
				uint8_t data[26];
			};
		};
	} __attribute__((packed));
} __attribute__((packed)) pd_msg;


/**
 * EPR PD message union
 * 
 * EPR requires the use of messages longer than 26 bytes, which the above union
 * cannot handle.  This union can handle messagaes up to 44 bytes long - 11
 * PDOs, which is the maximum number in an EPR Source Capabiltiies message.
 * 
 * The padding has been removed, as on ARM unaligned reads are not allowed to
 * occur.  With the padding, if you attempted to read from obj[0] an error
 * would occur as it would be 16 bits out of alignment.
 */
typedef union {
    uint8_t bytes[48];
    struct {
        uint16_t hdr;
        uint16_t exthdr;
        union {
            uint32_t obj[11];
            uint8_t data[44];
        } __attribute__((packed));
    } __attribute__((packed));
} __attribute__((packed)) epr_pd_msg;

#endif /* PDB_MSG_H */
