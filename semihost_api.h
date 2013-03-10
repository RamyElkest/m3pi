/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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
#ifndef MBED_SEMIHOST_H
#define MBED_SEMIHOST_H

#include "debug_frmwrk.h"
//#include "device.h"
//#include "toolchain.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_ID_LENGTH       32

#ifndef FILEHANDLE
typedef int FILEHANDLE;
#endif
#ifndef ssize_t
typedef int ssize_t;
#endif
#ifndef off_t
typedef long off_t;
#endif

inline int __semihost(int reason, const void *arg);

FILEHANDLE semihost_open(const char* name, int openmode);
int semihost_close (FILEHANDLE fh);
int semihost_read  (FILEHANDLE fh, unsigned char* buffer, unsigned int length, int mode);
int semihost_write (FILEHANDLE fh, const unsigned char* buffer, unsigned int length, int mode);
int semihost_ensure(FILEHANDLE fh);
long semihost_flen (FILEHANDLE fh);
int semihost_seek  (FILEHANDLE fh, long position);
int semihost_istty (FILEHANDLE fh);

int semihost_remove(const char *name);
int semihost_rename(const char *old_name, const char *new_name);

int semihost_uid(char *uid);
int semihost_reset(void);
int semihost_vbus(void);
int semihost_powerdown(void);
int semihost_exit(void);

int semihost_connected(void);
int semihost_disabledebug(void);

#ifdef __cplusplus
}
#endif

#endif

