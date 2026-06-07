/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi

    Narrow-band FM baseband generator.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include <stddef.h>


extern int nbfm_open(char *filename, size_t len);
extern int nbfm_get_samples(float *nbfm_buffer);
extern int nbfm_close(void);
