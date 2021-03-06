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
#include "LocalFileSystem.h"

#include "debug_frmwrk.h"
#include "semihost_api.h"
#include <string.h>
#include <stdio.h>

/* Extension to FINFO type defined in RTL.h (in Keil RL) - adds 'create time'. */
typedef struct {
    unsigned char  hr;   /* Hours    [0..23]                  */
    unsigned char  min;  /* Minutes  [0..59]                  */
    unsigned char  sec;  /* Seconds  [0..59]                  */
    unsigned char  day;  /* Day      [1..31]                  */
    unsigned char  mon;  /* Month    [1..12]                  */
    unsigned short year; /* Year     [1980..2107]             */
} FTIME;

typedef struct {         /* File Search info record           */
    char  name[32];      /* File name                         */
    long  size;          /* File size in bytes                */
    int   fileID;        /* System File Identification        */
    FTIME create_time;   /* Date & time file was created      */
    FTIME write_time;    /* Date & time of last write         */
} XFINFO;

#define RESERVED_FOR_USER_APPLICATIONS (0x100) /* 0x100 - 0x1ff */
#define USR_XFFIND (RESERVED_FOR_USER_APPLICATIONS + 0)

static int xffind (const char *pattern, XFINFO *info) {
    unsigned param[4];

    param[0] = (unsigned long)pattern;
    param[1] = (unsigned long)strlen(pattern);
    param[2] = (unsigned long)info;
    param[3] = (unsigned long)sizeof(XFINFO);

    return __semihost(USR_XFFIND, param);
}

#define OPEN_R          0
#define OPEN_B          1
#define OPEN_PLUS       2
#define OPEN_W          4
#define OPEN_A          8
#define OPEN_INVALID   -1

int posix_to_semihost_open_flags(int flags) {
    /* POSIX flags -> semihosting open mode */
    int openmode;
    if (flags & O_RDWR) {
        /* a plus mode */
        openmode = OPEN_PLUS;
        if (flags & O_APPEND) {
            openmode |= OPEN_A;
        } else if (flags & O_TRUNC) {
            openmode |= OPEN_W;
        } else {
            openmode |= OPEN_R;
        }
    } else if (flags & O_WRONLY) {
        /* write or append */
        if (flags & O_APPEND) {
            openmode = OPEN_A;
        } else {
            openmode = OPEN_W;
        }
    } else if (flags == O_RDONLY) {
        /* read mode */
        openmode = OPEN_R;
    } else {
        /* invalid flags */
        openmode = OPEN_INVALID;
    }

    return openmode;
}

FILEHANDLE local_file_open(const char* name, int flags) {
    int openmode = posix_to_semihost_open_flags(flags);
    if (openmode == OPEN_INVALID) {
        return (FILEHANDLE)NULL;
    }

    FILEHANDLE fh = semihost_open(name, openmode);
    if (fh == -1) {
        return (FILEHANDLE)NULL;
    }

    return fh;
}

void LocalFileHandle(FILEHANDLE fh) {
    _fh = fh;
    pos = 0;
}

int LocalFileHandle_close() {
	return semihost_close(_fh);
}

ssize_t LocalFileHandle_write(const void *buffer, size_t length) {
    ssize_t n = semihost_write(_fh, (const unsigned char*)buffer, length, 0); // number of characters not written
    n = length - n; // number of characters written
    pos += n;
    return n;
}

ssize_t LocalFileHandle_read(void *buffer, size_t length) {
    ssize_t n = semihost_read(_fh, (unsigned char*)buffer, length, 0); // number of characters not read
    n = length - n; // number of characters read
    pos += n;
    return n;
}

int LocalFileHandle_isatty() {
    return semihost_istty(_fh);
}

off_t LocalFileHandle_lseek(off_t position, int whence) {
    if (whence == SEEK_CUR) {
        position += pos;
    } else if (whence == SEEK_END) {
        position += semihost_flen(_fh);
    } /* otherwise SEEK_SET, so position is fine */

    /* Always seems to return -1, so just ignore for now. */
    semihost_seek(_fh, position);
    pos = position;
    return position;
}

int LocalFileHandle_fsync() {
    return semihost_ensure(_fh);
}

off_t LocalFileHandle_flen() {
    return semihost_flen(_fh);
}

XFINFO info;

void LocalDirHandle() {
	info.fileID = 0;
}

int closedir() {
	info.fileID = 0;
	return 0;
}

void rewinddir() {
	info.fileID = 0;
}

off_t telldir() {
	return info.fileID;
}

void seekdir(off_t offset) {
	info.fileID = offset;
}

FILEHANDLE LocalFileSystem_open(const char* name, int flags) {
    /* reject filenames with / in them */
	const char *tmp = name;
    for (; *tmp; tmp++) {
        if (*tmp == '/') {
			_DBG_("DISK ERROR: INVALID DIR NAME");
            return -1;
        }
    }

    int openmode = posix_to_semihost_open_flags(flags);
    if (openmode == OPEN_INVALID) {
		_DBG_("DISK ERROR: INVALID OPEN FLAG");
        return -1;
    }

    FILEHANDLE fh = semihost_open(name, openmode);
    if (fh == -1) {
		_DBG_("DISK ERROR: SEMIHOST FAILED TO OPEN");
        return -1;
    }
	LocalFileHandle(fh);
    return fh;
}

int LocalFileSystem_remove(const char *filename) {
    return semihost_remove(filename);
}
