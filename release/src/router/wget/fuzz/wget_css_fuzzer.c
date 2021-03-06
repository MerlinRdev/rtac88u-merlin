/*
 * Copyright(c) 2017-2018 Free Software Foundation, Inc.
 *
 * This file is part of GNU Wget.
 *
 * GNU Wget is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNU Wget is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Wget.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <config.h>

#include <sys/types.h>
#include <dirent.h> // opendir, readdir
#include <stdint.h> // uint8_t
#include <stdio.h>  // fmemopen
#include <string.h>  // strncmp
#include <stdlib.h>  // free
#include <fcntl.h>  // open flags
#include <unistd.h>  // close
#include <setjmp.h> // longjmp, setjmp

#include "wget.h"
#undef fopen_wgetrc

#ifdef __cplusplus
  extern "C" {
#endif
  #include "html-url.h"
  #include "css-url.h"

  // declarations for wget internal functions
  int main_wget(int argc, const char **argv);
  void cleanup(void);
  FILE *fopen_wget(const char *pathname, const char *mode);
  FILE *fopen_wgetrc(const char *pathname, const char *mode);
  void exit_wget(int status);
#ifdef __cplusplus
  }
#endif

#include "fuzzer.h"

static const uint8_t *g_data;
static size_t g_size;

FILE *fopen_wget(const char *pathname, const char *mode)
{
	return fopen("/dev/null", mode);
}

FILE *fopen_wgetrc(const char *pathname, const char *mode)
{
#ifdef HAVE_FMEMOPEN
	return fmemopen((void *) g_data, g_size, mode);
#else
	return NULL;
#endif
}

static jmp_buf jmpbuf;
#ifdef FUZZING
void exit_wget(int status)
{
	longjmp(jmpbuf, 1);
}
#else
void exit(int status)
{
	longjmp(jmpbuf, 1);
}
#endif

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
	if (size > 4096) // same as max_len = ... in .options file
		return 0;

	struct map_context ctx = {
		.text = (char *) data,
		.parent_base = strdup("https://x.y"),
		.document_file = NULL,
	};

	FILE *bak = stderr;
	stderr = fopen("/dev/null", "w");

	if (setjmp(jmpbuf))
		goto done;

	get_urls_css(&ctx, 0, size);
	free_urlpos(ctx.head);
	free((void *) ctx.parent_base);

done:
	fclose(stderr);
	stderr = bak;

	return 0;
}
