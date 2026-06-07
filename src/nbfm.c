/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi

    nbfm.c: generates a monaural narrow-band FM baseband signal without
    broadcast FM multiplex, stereo pilot, or RDS.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include <sndfile.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>


#define PI 3.141592654
#define NBFM_SAMPLE_RATE 228000.
#define NBFM_AUDIO_CUTOFF 3500.

#define FIR_HALF_SIZE 30
#define FIR_SIZE (2*FIR_HALF_SIZE-1)


static size_t nbfm_length;
static float low_pass_fir[FIR_HALF_SIZE];
static float downsample_factor;
static float *audio_buffer;
static int audio_index = 0;
static int audio_len = 0;
static float audio_pos;
static float fir_buffer[FIR_SIZE] = {0};
static int fir_index = 0;
static int channels;
static SNDFILE *inf;



static float *alloc_empty_buffer(size_t length) {
    float *p = malloc(length * sizeof(float));
    if(p == NULL) return NULL;

    bzero(p, length * sizeof(float));

    return p;
}




static void create_low_pass_filter(int in_samplerate) {
    float cutoff_freq = NBFM_AUDIO_CUTOFF;
    if(in_samplerate/2 < cutoff_freq) cutoff_freq = in_samplerate/2 * .8;

    low_pass_fir[FIR_HALF_SIZE-1] = 2 * cutoff_freq / NBFM_SAMPLE_RATE / 2;

    for(int i=1; i<FIR_HALF_SIZE; i++) {
        low_pass_fir[FIR_HALF_SIZE-1-i] =
            sin(2 * PI * cutoff_freq * i / NBFM_SAMPLE_RATE) / (PI * i) *
            (.54 - .46 * cos(2 * PI * (i + FIR_HALF_SIZE) / (2 * FIR_HALF_SIZE)));
    }

    printf("Created NBFM low-pass FIR filter for audio, with cutoff at %.1f Hz\n", cutoff_freq);
}




int nbfm_open(char *filename, size_t len) {
    nbfm_length = len;
    audio_index = 0;
    audio_len = 0;
    fir_index = 0;
    audio_buffer = NULL;
    bzero(fir_buffer, sizeof(fir_buffer));

    if(filename != NULL) {
        SF_INFO sfinfo;

        if(filename[0] == '-') {
            if(! (inf = sf_open_fd(fileno(stdin), SFM_READ, &sfinfo, 0))) {
                fprintf(stderr, "Error: could not open stdin for audio input.\n");
                return -1;
            } else {
                printf("Using stdin for NBFM audio input.\n");
            }
        } else {
            if(! (inf = sf_open(filename, SFM_READ, &sfinfo))) {
                fprintf(stderr, "Error: could not open input file %s.\n", filename);
                return -1;
            } else {
                printf("Using NBFM audio file: %s\n", filename);
            }
        }

        int in_samplerate = sfinfo.samplerate;
        downsample_factor = NBFM_SAMPLE_RATE / in_samplerate;

        printf("Input: %d Hz, upsampling factor: %.2f\n", in_samplerate, downsample_factor);

        channels = sfinfo.channels;
        if(channels > 1) {
            printf("%d channels, mixing down to monophonic NBFM.\n", channels);
        } else {
            printf("1 channel, monophonic NBFM operation.\n");
        }

        create_low_pass_filter(in_samplerate);

        audio_pos = downsample_factor;
        audio_buffer = alloc_empty_buffer(nbfm_length * channels);
        if(audio_buffer == NULL) return -1;
    } else {
        inf = NULL;
    }

    return 0;
}




int nbfm_get_samples(float *nbfm_buffer) {
    bzero(nbfm_buffer, nbfm_length * sizeof(float));

    if(inf == NULL) return 0;

    for(int i=0; i<nbfm_length; i++) {
        if(audio_pos >= downsample_factor) {
            audio_pos -= downsample_factor;

            if(audio_len == 0) {
                for(int j=0; j<2; j++) {
                    audio_len = sf_readf_float(inf, audio_buffer, nbfm_length);
                    if(audio_len < 0) {
                        fprintf(stderr, "Error reading audio\n");
                        return -1;
                    }
                    if(audio_len == 0) {
                        if(sf_seek(inf, 0, SEEK_SET) < 0) {
                            fprintf(stderr, "Could not rewind in audio file, terminating\n");
                            return -1;
                        }
                    } else {
                        break;
                    }
                }
                audio_index = 0;
                if(audio_len > 0) audio_len--;
            } else {
                audio_index += channels;
                audio_len--;
            }
        }

        float mono_sample = audio_buffer[audio_index];
        if(channels > 1) {
            mono_sample = 0;
            for(int channel_index=0; channel_index<channels; channel_index++) {
                mono_sample += audio_buffer[audio_index + channel_index];
            }
            mono_sample /= channels;
        }

        fir_buffer[fir_index] = mono_sample;
        fir_index++;
        if(fir_index >= FIR_SIZE) fir_index = 0;

        float out_mono = 0;
        int increasing_fir_index = fir_index;
        int decreasing_fir_index = fir_index;
        for(int filter_index=0; filter_index<FIR_HALF_SIZE; filter_index++) {
            decreasing_fir_index--;
            if(decreasing_fir_index < 0) decreasing_fir_index = FIR_SIZE-1;
            out_mono +=
                low_pass_fir[filter_index] *
                    (fir_buffer[increasing_fir_index] + fir_buffer[decreasing_fir_index]);
            increasing_fir_index++;
            if(increasing_fir_index >= FIR_SIZE) increasing_fir_index = 0;
        }

        nbfm_buffer[i] = 10.0 * out_mono;
        audio_pos++;
    }

    return 0;
}




int nbfm_close(void) {
    if(inf != NULL && sf_close(inf)) {
        fprintf(stderr, "Error closing audio file");
    }

    if(audio_buffer != NULL) free(audio_buffer);
    audio_buffer = NULL;
    inf = NULL;

    return 0;
}
