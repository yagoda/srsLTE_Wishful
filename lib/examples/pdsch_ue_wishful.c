/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
//#include "pipe.h"
//#include "parson.h"
#include "srslte/srslte.h"
#include "srslte/phy/io/filesink.h"
#define WISHFUL
#define ENABLE_AGC_DEFAULT

#ifndef DISABLE_RF
#include "srslte/phy/rf/rf.h"
#include "srslte/phy/rf/rf_utils.h"
srslte_rf_t rf; 
cell_search_cfg_t cell_detect_config = {
  SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
  SRSLTE_DEFAULT_MAX_FRAMES_PSS, 
  SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
  0
};

#else
#warning Compiling pdsch_ue with no RF support
#endif

//#define STDOUT_COMPACT

#ifndef DISABLE_GRAPHICS
#include "srsgui/srsgui.h"
void init_plots();
pthread_t plot_thread; 
sem_t plot_sem; 
uint32_t plot_sf_idx=0;
bool plot_track = true; 
#endif


#ifdef WISHFUL
void init_wishful_receive(pipe_t* command_pipe);
void init_wishful_send(pipe_t* command_pipe);

pthread_t wishful_thread_send;
pthread_t wishful_thread_receive;
sem_t wishful_sem;

srslte_netsource_t wishful_command_server;
srslte_netsink_t wishful_metric_client;

srslte_filesink_t CSI_sink;
char *CSI_output_file_name;

typedef struct {
    bool wants_metric;
    bool make_config;
    float config_value;
    int which_metric;
    int which_config;

} wishful_command;

typedef struct {
    int which_metric;
    float metric_value;
    bool is_reconfig;
    int which_reconfig;
    float reconfig_value;
}wishful_response;

enum parameter_state {FREQ, GAIN};
#endif


#define PRINT_CHANGE_SCHEDULIGN

//#define CORRECT_SAMPLE_OFFSET

/**********************************************************************
 *  Program arguments processing
 ***********************************************************************/
typedef struct {
  int nof_subframes;
  int cpu_affinity;
  bool disable_plots;
  bool disable_plots_except_constellation;
  bool disable_cfo; 
  uint32_t time_offset; 
  int force_N_id_2;
  uint16_t rnti;
  char *input_file_name;
  int file_offset_time; 
  float file_offset_freq;
  uint32_t file_nof_prb;
  uint32_t file_nof_ports;
  uint32_t file_cell_id;
  char *rf_args;
  uint32_t rf_nof_rx_ant;
  double rf_freq; 
  float rf_gain;
  int net_port; 
  char *net_address; 
  int net_port_signal; 
  char *net_address_signal;
  int decimate;
  //WISHFUL
  char *equalizer;
  int max_turbo_its;
  sss_alg_t sss_algorithm;
  srslte_chest_dl_noise_alg_t noise_alg;
  float snr_ema_coeff;
  float cfo_tol;
  char *csi_output_file;
 
}prog_args_t;



void args_default(prog_args_t *args) {
  args->disable_plots = false; 
  args->disable_plots_except_constellation = false; 
  args->nof_subframes = -1;
  args->rnti = SRSLTE_SIRNTI;
  args->force_N_id_2 = -1; // Pick the best
  args->input_file_name = NULL;
  args->disable_cfo = false; 
  args->time_offset = 0; 
  args->file_nof_prb = 25; 
  args->file_nof_ports = 1; 
  args->file_cell_id = 0; 
  args->file_offset_time = 0; 
  args->file_offset_freq = 0; 
  args->rf_args = "";
  args->rf_freq = -1.0;
#ifdef ENABLE_AGC_DEFAULT
  args->rf_gain = -1.0; 
#else
  args->rf_gain = 50.0; 
#endif
  args->rf_nof_rx_ant = 1;
  args->net_port = 1111; 
  args->net_address = "127.0.0.1";
  args->net_port_signal = -1; 
  args->net_address_signal = "127.0.0.1";
  args->decimate = 0;
  args->cpu_affinity = -1;
  //WISHFUL
  args->equalizer = "zf";
  args->max_turbo_its = 4;
  args->sss_algorithm = SSS_FULL;
  args->noise_alg = SRSLTE_NOISE_ALG_REFS;
  args->snr_ema_coeff = 0.1;
  args->cfo_tol = 50.0;
  args->csi_output_file = "CSI.bin";
}

void usage(prog_args_t *args, char *prog) {
  printf("Usage: %s [agpPoOcildDnruv] -f rx_frequency (in Hz) | -i input_file\n", prog);
#ifndef DISABLE_RF
  printf("\t-a RF args [Default %s]\n", args->rf_args);
  printf("\t-A Number of RX antennas [Default %d]\n", args->rf_nof_rx_ant);

#ifdef ENABLE_AGC_DEFAULT
  printf("\t-g RF fix RX gain [Default AGC]\n");
#else
  printf("\t-g Set RX gain [Default %.1f dB]\n", args->rf_gain);
#endif  
#else
  printf("\t   RF is disabled.\n");
#endif
  printf("\t-i input_file [Default use RF board]\n");
  printf("\t-o offset frequency correction (in Hz) for input file [Default %.1f Hz]\n", args->file_offset_freq);
  printf("\t-O offset samples for input file [Default %d]\n", args->file_offset_time);
  printf("\t-p nof_prb for input file [Default %d]\n", args->file_nof_prb);
  printf("\t-P nof_ports for input file [Default %d]\n", args->file_nof_ports);
  printf("\t-c cell_id for input file [Default %d]\n", args->file_cell_id);
  printf("\t-r RNTI in Hex [Default 0x%x]\n",args->rnti);
  printf("\t-l Force N_id_2 [Default best]\n");
  printf("\t-C Disable CFO correction [Default %s]\n", args->disable_cfo?"Disabled":"Enabled");
  printf("\t-t Add time offset [Default %d]\n", args->time_offset);
#ifndef DISABLE_GRAPHICS
  printf("\t-d disable plots [Default enabled]\n");
  printf("\t-D disable all but constellation plots [Default enabled]\n");
#else
  printf("\t plots are disabled. Graphics library not available\n");
#endif
  printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
  printf("\t-s remote UDP port to send input signal (-1 does nothing with it) [Default %d]\n", args->net_port_signal);
  printf("\t-S remote UDP address to send input signal [Default %s]\n", args->net_address_signal);
  printf("\t-u remote TCP port to send data (-1 does nothing with it) [Default %d]\n", args->net_port);
  printf("\t-U remote TCP address to send data [Default %s]\n", args->net_address);
  printf("\t-v [set srslte_verbose to debug, default none]\n");
}

void parse_args(prog_args_t *args, int argc, char **argv) {
  int opt;
  args_default(args);
  while ((opt = getopt(argc, argv, "aAoglipPcOCtdDnvrfuUsSeyENXTeZ")) != -1) {
    switch (opt) {
    case 'i':
      args->input_file_name = argv[optind];
      break;
    case 'p':
      args->file_nof_prb = atoi(argv[optind]);
      break;
    case 'P':
      args->file_nof_ports = atoi(argv[optind]);
      break;
    case 'o':
      args->file_offset_freq = atof(argv[optind]);
      break;
    case 'O':
      args->file_offset_time = atoi(argv[optind]);
      break;
    case 'c':
      args->file_cell_id = atoi(argv[optind]);
      break;
    case 'a':
      args->rf_args = argv[optind];
      break;
    case 'A':
      args->rf_nof_rx_ant = atoi(argv[optind]);
      break;
    case 'g':
      args->rf_gain = atof(argv[optind]);
      break;
    case 'C':
      args->disable_cfo = true;
      break;
    case 't':
      args->time_offset = atoi(argv[optind]);
      break;
    case 'f':
      args->rf_freq = strtod(argv[optind], NULL);
      break;
    case 'n':
      args->nof_subframes = atoi(argv[optind]);
      break;
    case 'r':
      args->rnti = strtol(argv[optind], NULL, 16);
      break;
    case 'l':
      args->force_N_id_2 = atoi(argv[optind]);
      break;
    case 'u':
      args->net_port = atoi(argv[optind]);
      break;
    case 'U':
      args->net_address = argv[optind];
      break;
    case 's':
      args->net_port_signal = atoi(argv[optind]);
      break;
    case 'S':
      args->net_address_signal = argv[optind];
      break;
    case 'd':
      args->disable_plots = true;
      break;
    case 'D':
      args->disable_plots_except_constellation = true;
      break;
    case 'v':
      srslte_verbose++;
      break;
    case 'e':
      args->equalizer = argv[optind];
      break;
    case 'T':
      args->max_turbo_its =  atoi(argv[optind]);
    break;
    case 'X':
      args->sss_algorithm = atoi(argv[optind]);
    break;
    case 'N':
      args->noise_alg = atoi(argv[optind]);
    break;
    case 'E':
      args->snr_ema_coeff = atof(argv[optind]);
    break;
    case 'y':
      args->cfo_tol = atof(argv[optind]);
    break;
    case 'H':
      args->csi_output_file = argv[optind];
    break;
    case 'Z':
      args->decimate = atoi(argv[optind]);
    break;  
    
    default:
      usage(args, argv[0]);
      exit(-1);
    }
  }
  if (args->rf_freq < 0 && args->input_file_name == NULL) {
    usage(args, argv[0]);
    exit(-1);
  }
}
/**********************************************************************/

/* TODO: Do something with the output data */
uint8_t data[20000];

bool go_exit = false; 
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}
cf_t *sf_buffer[2] = {NULL, NULL}; 
#ifndef DISABLE_RF
int srslte_rf_recv_wrapper(void *h, cf_t *data[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  void *ptr[SRSLTE_MAX_PORTS];
  for (int i=0;i<SRSLTE_MAX_PORTS;i++) {
    ptr[i] = data[i];
  }
  return srslte_rf_recv_with_time_multi(h, ptr, nsamples, true, NULL, NULL);
}



double srslte_rf_set_rx_gain_th_wrapper_(void *h, double f) {
  return srslte_rf_set_rx_gain_th((srslte_rf_t*) h, f);
}

#endif

extern float mean_exec_time;

enum receiver_state {DECODE_MIB, DECODE_PDSCH} state; 

enum metric_state {CFO, SNR, RSRP, RSRQ, NOISE, CSI, N_FRAMES, PDSCH_MISS, PDCCH_MISS, MOD, TBS};

srslte_ue_dl_t ue_dl; 
srslte_ue_sync_t ue_sync; 
prog_args_t prog_args;

//////////////////////////////
typedef struct{
  uint32_t nframes;
  float rsrp; 
  float rsrq;
  float noise;
  bool  decode_pdsch;
  bool  CSI;
  float pdsch_miss;
  float pdcch_miss;
  }metrics;
  
  metrics phy_metrics;

  
///////////////////////////
uint32_t sfn = 0; // system frame number
srslte_netsink_t net_sink, net_sink_signal; 

int main(int argc, char **argv) {
  
#ifdef WISHFUL
  pipe_t* command_pipe = pipe_new(sizeof(wishful_command), 0);
#endif
  int ret; 
  int decimate = 1;
  srslte_cell_t cell;  
  int64_t sf_cnt;
  srslte_ue_mib_t ue_mib; 
  uint32_t nof_trials = 0; 
  int n; 
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  int sfn_offset;
  float cfo = 0; 
  
  parse_args(&prog_args, argc, argv);
  
  for (int i=0;i<prog_args.rf_nof_rx_ant;i++) {
    sf_buffer[i] = srslte_vec_malloc(3*sizeof(cf_t)*SRSLTE_SF_LEN_PRB(cell.nof_prb));
  }
  
  //WISHFUL
  ue_sync.sss_algorithm = prog_args.sss_algorithm;
  ue_sync.cfo_tol = prog_args.cfo_tol;
  ue_dl.max_turbo_its = prog_args.max_turbo_its;
  ue_dl.noise_alg = prog_args.noise_alg;
  
  if(strcmp(prog_args.equalizer, "zf") == 0) {
    ue_dl.zf_equalizer = 1;
  }
  else {
    ue_dl.zf_equalizer = 0;
  }
  //////
  #ifdef WISHFUL
  init_wishful_receive(command_pipe);
  sleep(1);
  init_wishful_send(command_pipe);
  sleep(1);
#endif 
  
  if (prog_args.net_port > 0) {
    if (srslte_netsink_init(&net_sink, prog_args.net_address, prog_args.net_port, SRSLTE_NETSINK_TCP)) {
      fprintf(stderr, "Error initiating UDP socket to %s:%d\n", prog_args.net_address, prog_args.net_port);
      exit(-1);
    }
    srslte_netsink_set_nonblocking(&net_sink);
  }
  if (prog_args.net_port_signal > 0) {
    if (srslte_netsink_init(&net_sink_signal, prog_args.net_address_signal, 
      prog_args.net_port_signal, SRSLTE_NETSINK_UDP)) {
      fprintf(stderr, "Error initiating UDP socket to %s:%d\n", prog_args.net_address_signal, prog_args.net_port_signal);
      exit(-1);
    }
    srslte_netsink_set_nonblocking(&net_sink_signal);
  }
  
#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    
    printf("Opening RF device with %d RX antennas...\n", prog_args.rf_nof_rx_ant);
    if (srslte_rf_open_multi(&rf, prog_args.rf_args, prog_args.rf_nof_rx_ant)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }

    /* Set receiver gain */
    if (prog_args.rf_gain > 0) {
      srslte_rf_set_rx_gain(&rf, prog_args.rf_gain);      
    } else {
      printf("Starting AGC thread...\n");
      if (srslte_rf_start_gain_thread(&rf, false)) {
        fprintf(stderr, "Error opening rf\n");
        exit(-1);
      }
      srslte_rf_set_rx_gain(&rf, 50);      
      cell_detect_config.init_agc = 50; 
    }
    
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigprocmask(SIG_UNBLOCK, &sigset, NULL);
    signal(SIGINT, sig_int_handler);
    
    srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

    /* set receiver frequency */
    printf("Tunning receiver to %.3f MHz\n", prog_args.rf_freq/1000000);
    srslte_rf_set_rx_freq(&rf, prog_args.rf_freq);
    srslte_rf_rx_wait_lo_locked(&rf);

    uint32_t ntrial=0; 
    do {
      ret = rf_search_and_decode_mib(&rf,prog_args.rf_nof_rx_ant , &cell_detect_config, prog_args.force_N_id_2, &cell, &cfo);
      if (ret < 0) {
        fprintf(stderr, "Error searching for cell\n");
        exit(-1); 
      } else if (ret == 0 && !go_exit) {
        printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
      }      
    } while (ret == 0 && !go_exit); 
    
    if (go_exit) {
      srslte_rf_close(&rf);    
      exit(0);
    }

    //srslte_rf_stop_rx_stream(&rf);
    //srslte_rf_flush_buffer(&rf);    

    /* set sampling frequency */
    int srate = srslte_sampling_freq_hz(cell.nof_prb);    
    if (srate != -1) {  
      if (srate < 10e6) {          
        srslte_rf_set_master_clock_rate(&rf, 4*srate);        
      } else {
        srslte_rf_set_master_clock_rate(&rf, srate);        
      }
      printf("Setting sampling rate %.2f MHz\n", (float) srate/1000000);
      float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
      if (srate_rf != srate) {
        fprintf(stderr, "Could not set sampling rate\n");
        exit(-1);
      }
    } else {
      fprintf(stderr, "Invalid number of PRB %d\n", cell.nof_prb);
      exit(-1);
    }

    INFO("Stopping RF and flushing buffer...\r",0);
    srslte_rf_stop_rx_stream(&rf);
    srslte_rf_flush_buffer(&rf);    
  }
#endif
  
  /* If reading from file, go straight to PDSCH decoding. Otherwise, decode MIB first */
  if (prog_args.input_file_name) {
    /* preset cell configuration */
    cell.id = prog_args.file_cell_id; 
    cell.cp = SRSLTE_CP_NORM; 
    cell.phich_length = SRSLTE_PHICH_NORM;
    cell.phich_resources = SRSLTE_PHICH_R_1;
    cell.nof_ports = prog_args.file_nof_ports; 
    cell.nof_prb = prog_args.file_nof_prb; 
    
    if (srslte_ue_sync_init_file(&ue_sync, prog_args.file_nof_prb, 
      prog_args.input_file_name, prog_args.file_offset_time, prog_args.file_offset_freq)) {
      fprintf(stderr, "Error initiating ue_sync\n");
      exit(-1); 
    }

  } else {
#ifndef DISABLE_RF
      if(prog_args.decimate)
      {
          if(prog_args.decimate > 4 || prog_args.decimate < 0)
          {
              printf("Invalid decimation factor, setting to 1 \n");
          }
          else
          {
              decimate = prog_args.decimate;
             //ue_sync.decimate = prog_args.decimate;
          }
      }
    if (srslte_ue_sync_init_multi_decim(&ue_sync, cell, srslte_rf_recv_wrapper, prog_args.rf_nof_rx_ant, (void*) &rf, decimate)) {

      fprintf(stderr, "Error initiating ue_sync\n");
      exit(-1); 
    }
#endif
  }
  ue_mib.noise_alg = prog_args.noise_alg;
  if (srslte_ue_mib_init(&ue_mib, cell)) {
    fprintf(stderr, "Error initaiting UE MIB decoder\n");
    exit(-1);
  }    

  if (srslte_ue_dl_init_multi(&ue_dl, cell, prog_args.rf_nof_rx_ant)) {  // This is the User RNTI
    fprintf(stderr, "Error initiating UE downlink processing module\n");
    exit(-1);
  }
  
  /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
  srslte_ue_dl_set_rnti(&ue_dl, prog_args.rnti); 
  
  /* Initialize subframe counter */
  sf_cnt = 0;


#ifndef DISABLE_GRAPHICS
  if (!prog_args.disable_plots) {
    init_plots(cell);    
    sleep(1);
  }
#endif


#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_rf_start_rx_stream(&rf);    
  }
#endif
    
  // Variables for measurements 
  
  phy_metrics.nframes = 0;
  phy_metrics.rsrp = 0.0;
  phy_metrics.rsrq = 0.0;
  phy_metrics.noise = 0.0;
  phy_metrics.decode_pdsch = false;
  //uint32_t nframes=0;
  //float rsrp=0.0, rsrq=0.0, noise=0.0;
  //bool decode_pdsch = false; 

#ifndef DISABLE_RF
  if (prog_args.rf_gain < 0) {
    srslte_ue_sync_start_agc(&ue_sync, srslte_rf_set_rx_gain_th_wrapper_, cell_detect_config.init_agc);
  }
#endif
#ifdef PRINT_CHANGE_SCHEDULIGN
  srslte_ra_dl_dci_t old_dl_dci; 
  bzero(&old_dl_dci, sizeof(srslte_ra_dl_dci_t));
#endif
  
  ue_sync.correct_cfo = !prog_args.disable_cfo;
  
  // Set initial CFO for ue_sync
  srslte_ue_sync_set_cfo(&ue_sync, cfo); 
  
  srslte_pbch_decode_reset(&ue_mib.pbch);
            
  INFO("\nEntering main loop...\n\n", 0);
  /* Main loop */
  while (!go_exit && (sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1)) {
    ret = srslte_ue_sync_zerocopy_multi(&ue_sync, sf_buffer);
    //ret = srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
    if (ret < 0) {
      fprintf(stderr, "Error calling srslte_ue_sync_work()\n");
    }

#ifdef CORRECT_SAMPLE_OFFSET
    float sample_offset = (float) srslte_ue_sync_get_last_sample_offset(&ue_sync)+srslte_ue_sync_get_sfo(&ue_sync)/1000; 
    srslte_ue_dl_set_sample_offset(&ue_dl, sample_offset);
#endif
    
    /* srslte_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
    if (ret == 1) {
      switch (state) {
        case DECODE_MIB:
            //printf("decode mib\n");
          if (srslte_ue_sync_get_sfidx(&ue_sync) == 0) {
            n = srslte_ue_mib_decode(&ue_mib, sf_buffer[0], bch_payload, NULL, &sfn_offset);
            if (n < 0) {
              fprintf(stderr, "Error decoding UE MIB\n");
              exit(-1);
            } else if (n == SRSLTE_UE_MIB_FOUND) {             
              srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
              srslte_cell_fprint(stdout, &cell, sfn);
              printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
              sfn = (sfn + sfn_offset)%1024; 
              state = DECODE_PDSCH;               
            }
          }
          break;
        case DECODE_PDSCH:
            //printf("decode pdsch\n");
          if (prog_args.rnti != SRSLTE_SIRNTI) {
            phy_metrics.decode_pdsch = true;             
          } else {
            /* We are looking for SIB1 Blocks, search only in appropiate places */
            if ((srslte_ue_sync_get_sfidx(&ue_sync) == 5 && (sfn%2)==0)) {
              phy_metrics.decode_pdsch = true; 
            } else {
              phy_metrics.decode_pdsch = false; 
            }
          }
          if (phy_metrics.decode_pdsch) {            
            INFO("Attempting DL decode SFN=%d\n", sfn);
            n = srslte_ue_dl_decode_multi(&ue_dl, 
                                    sf_buffer, 
                                    data, 
                                    sfn*10+srslte_ue_sync_get_sfidx(&ue_sync));
          
            if (n < 0) {
             // fprintf(stderr, "Error decoding UE DL\n");fflush(stdout);
            } else if (n > 0) {
              
              /* Send data if socket active */
              if (prog_args.net_port > 0) {
                srslte_netsink_write(&net_sink, data, 1+(n-1)/8);
              }
              
              #ifdef PRINT_CHANGE_SCHEDULIGN
              if (ue_dl.dl_dci.mcs_idx         != old_dl_dci.mcs_idx           || 
                  memcmp(&ue_dl.dl_dci.type0_alloc, &old_dl_dci.type0_alloc, sizeof(srslte_ra_type0_t)) ||
                  memcmp(&ue_dl.dl_dci.type1_alloc, &old_dl_dci.type1_alloc, sizeof(srslte_ra_type1_t)) ||
                  memcmp(&ue_dl.dl_dci.type2_alloc, &old_dl_dci.type2_alloc, sizeof(srslte_ra_type2_t)))
              {
                memcpy(&old_dl_dci, &ue_dl.dl_dci, sizeof(srslte_ra_dl_dci_t));
                fflush(stdout);
                printf("Format: %s\n", srslte_dci_format_string(ue_dl.dci_format));
                srslte_ra_pdsch_fprint(stdout, &old_dl_dci, cell.nof_prb);
                srslte_ra_dl_grant_fprint(stdout, &ue_dl.pdsch_cfg.grant);
              }
              #endif

            } 
                                    
            nof_trials++; 
            
            phy_metrics.rsrq = SRSLTE_VEC_EMA(srslte_chest_dl_get_rsrq(&ue_dl.chest), phy_metrics.rsrq, prog_args.snr_ema_coeff);
            phy_metrics.rsrp = SRSLTE_VEC_EMA(srslte_chest_dl_get_rsrp(&ue_dl.chest), phy_metrics.rsrp, prog_args.snr_ema_coeff);      
            phy_metrics.noise = SRSLTE_VEC_EMA(srslte_chest_dl_get_noise_estimate(&ue_dl.chest), phy_metrics.noise, prog_args.snr_ema_coeff);      
            phy_metrics.nframes++;
            if (isnan(phy_metrics.rsrq)) {
              phy_metrics.rsrq = 0; 
            }
            if (isnan(phy_metrics.noise)) {
              phy_metrics.noise = 0; 
            }
            if (isnan(phy_metrics.rsrp)) {
              phy_metrics.rsrp = 0; 
            }        
          }
          phy_metrics.pdsch_miss = (float) 100*ue_dl.pkt_errors/ue_dl.pkts_total;
          phy_metrics.pdcch_miss = 100*(1-(float) ue_dl.nof_detected/nof_trials);
          // Plot and Printf
          if (srslte_ue_sync_get_sfidx(&ue_sync) == 5) {
            float gain = prog_args.rf_gain; 
            if (gain < 0) {
              gain = 10*log10(srslte_agc_get_gain(&ue_sync.agc)); 
            }
            /*
            printf("CFO: %+6.2f kHz, "
                   "SNR: %4.1f dB, "
                   "PDCCH-Miss: %5.2f%%, PDSCH-BLER: %5.2f%%\r",
                   
                  srslte_ue_sync_get_cfo(&ue_sync)/1000,
                  10*log10(phy_metrics.rsrp/phy_metrics.noise), 
                  100*(1-(float) ue_dl.nof_detected/nof_trials), 
                  (float) 100*ue_dl.pkt_errors/ue_dl.pkts_total);    */                    
          }
          break;
      }
      if (srslte_ue_sync_get_sfidx(&ue_sync) == 9) {
        sfn++; 
        if (sfn == 1024) {
          sfn = 0; 
          //printf("\n");
          ue_dl.pkt_errors = 0; 
          ue_dl.pkts_total = 0; 
          ue_dl.nof_detected = 0;           
          nof_trials = 0; 
        } 
      }
      
      #ifndef DISABLE_GRAPHICS
      if (!prog_args.disable_plots) {
        if ((sfn%4) == 0 && phy_metrics.decode_pdsch) {
          plot_sf_idx = srslte_ue_sync_get_sfidx(&ue_sync);
          plot_track = true;
          sem_post(&plot_sem);
        }
      }
      #endif
    } else if (ret == 0) {
      printf("Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\r", 
        srslte_sync_get_peak_value(&ue_sync.sfind), 
        ue_sync.frame_total_cnt, ue_sync.state);      
      #ifndef DISABLE_GRAPHICS
      if (!prog_args.disable_plots) {
        plot_sf_idx = srslte_ue_sync_get_sfidx(&ue_sync);
        plot_track = false; 
        sem_post(&plot_sem);                
      }
      #endif
    }
        
    sf_cnt++;                  
  } // Main loop
  
#ifndef DISABLE_GRAPHICS
  if (!prog_args.disable_plots) {
    if (!pthread_kill(plot_thread, 0)) {
      pthread_kill(plot_thread, SIGHUP);
      pthread_join(plot_thread, NULL);    
    }
  }
#endif
  srslte_ue_dl_free(&ue_dl);
  srslte_ue_sync_free(&ue_sync);
  
#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_ue_mib_free(&ue_mib);
    srslte_rf_close(&rf);    
  }
#endif
  printf("\nBye\n");
  exit(0);
}






  

/**********************************************************************
 *  Plotting Functions
 ***********************************************************************/
#ifndef DISABLE_GRAPHICS


plot_real_t p_sync, pce;
plot_scatter_t  pscatequal, pscatequal_pdcch;

float tmp_plot[110*15*2048];
float tmp_plot2[110*15*2048];
float tmp_plot3[110*15*2048];

void *plot_thread_run(void *arg) {
  int i;
  uint32_t nof_re = SRSLTE_SF_LEN_RE(ue_dl.cell.nof_prb, ue_dl.cell.cp);
    
  
  sdrgui_init();
  
  plot_scatter_init(&pscatequal);
  plot_scatter_setTitle(&pscatequal, "PDSCH - Equalized Symbols");
  plot_scatter_setXAxisScale(&pscatequal, -4, 4);
  plot_scatter_setYAxisScale(&pscatequal, -4, 4);

  plot_scatter_addToWindowGrid(&pscatequal, (char*)"pdsch_ue", 0, 0);

  if (!prog_args.disable_plots_except_constellation) {
    plot_real_init(&pce);
    plot_real_setTitle(&pce, "Channel Response - Magnitude");
    plot_real_setLabels(&pce, "Index", "dB");
    plot_real_setYAxisScale(&pce, -40, 40);
    
    plot_real_init(&p_sync);
    plot_real_setTitle(&p_sync, "PSS Cross-Corr abs value");
    plot_real_setYAxisScale(&p_sync, 0, 1);

    plot_scatter_init(&pscatequal_pdcch);
    plot_scatter_setTitle(&pscatequal_pdcch, "PDCCH - Equalized Symbols");
    plot_scatter_setXAxisScale(&pscatequal_pdcch, -4, 4);
    plot_scatter_setYAxisScale(&pscatequal_pdcch, -4, 4);

    plot_real_addToWindowGrid(&pce, (char*)"pdsch_ue",    0, 1);
    plot_real_addToWindowGrid(&pscatequal_pdcch, (char*)"pdsch_ue", 1, 0);
    plot_real_addToWindowGrid(&p_sync, (char*)"pdsch_ue", 1, 1);
  }
  
  while(1) {
    sem_wait(&plot_sem);
    
    uint32_t nof_symbols = ue_dl.pdsch_cfg.nbits.nof_re;
    if (!prog_args.disable_plots_except_constellation) {      
      for (i = 0; i < nof_re; i++) {
        tmp_plot[i] = 20 * log10f(cabsf(ue_dl.sf_symbols[i]));
        if (isinf(tmp_plot[i])) {
          tmp_plot[i] = -80;
        }
      }
      int sz = srslte_symbol_sz(ue_dl.cell.nof_prb);
      bzero(tmp_plot2, sizeof(float)*sz);
      int g = (sz - 12*ue_dl.cell.nof_prb)/2;
      for (i = 0; i < 12*ue_dl.cell.nof_prb; i++) {
        tmp_plot2[g+i] = 20 * log10(cabs(ue_dl.ce[0][i]));
        if (isinf(tmp_plot2[g+i])) {
          tmp_plot2[g+i] = -80;
        }
      }
      plot_real_setNewData(&pce, tmp_plot2, sz);        
      
      if (!prog_args.input_file_name) {
        if (plot_track) {
          srslte_pss_synch_t *pss_obj = srslte_sync_get_cur_pss_obj(&ue_sync.strack);
          int max = srslte_vec_max_fi(pss_obj->conv_output_avg, pss_obj->frame_size+pss_obj->fft_size-1);
          srslte_vec_sc_prod_fff(pss_obj->conv_output_avg, 
                          1/pss_obj->conv_output_avg[max], 
                          tmp_plot2, 
                          pss_obj->frame_size+pss_obj->fft_size-1);        
          plot_real_setNewData(&p_sync, tmp_plot2, pss_obj->frame_size);        
        } else {
          int max = srslte_vec_max_fi(ue_sync.sfind.pss.conv_output_avg, ue_sync.sfind.pss.frame_size+ue_sync.sfind.pss.fft_size-1);
          srslte_vec_sc_prod_fff(ue_sync.sfind.pss.conv_output_avg, 
                          1/ue_sync.sfind.pss.conv_output_avg[max], 
                          tmp_plot2, 
                          ue_sync.sfind.pss.frame_size+ue_sync.sfind.pss.fft_size-1);        
          plot_real_setNewData(&p_sync, tmp_plot2, ue_sync.sfind.pss.frame_size);        
        }
        
      }
      
      plot_scatter_setNewData(&pscatequal_pdcch, ue_dl.pdcch.d, 36*ue_dl.pdcch.nof_cce);
    }
    
    plot_scatter_setNewData(&pscatequal, ue_dl.pdsch.d, nof_symbols);
    
    if (plot_sf_idx == 1) {
      if (prog_args.net_port_signal > 0) {
        srslte_netsink_write(&net_sink_signal, &sf_buffer[srslte_ue_sync_sf_len(&ue_sync)/7], 
                            srslte_ue_sync_sf_len(&ue_sync)); 
      }
    }

  }
  
  return NULL;
}

void init_plots() {

  if (sem_init(&plot_sem, 0, 0)) {
    perror("sem_init");
    exit(-1);
  }
  
  pthread_attr_t attr;
  struct sched_param param;
  param.sched_priority = 0;  
  pthread_attr_init(&attr);
  pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
  pthread_attr_setschedparam(&attr, &param);
  if (pthread_create(&plot_thread, NULL, plot_thread_run, NULL)) {
    perror("pthread_create");
    exit(-1);
  }  
}

#endif


//********************WISHFUL FUNCTIONALITY**************************//
#ifdef WISHFUL
void * wishful_thread_receive_run(void *args)
{
   
    pipe_t* command_pipe = args;
    
    int portNo = 4321;
    //char buf[256];
    //int ret = srslte_netsource_init(&wishful_command_server, prog_args.net_address, prog_args.net_port, SRSLTE_NETSOURCE_TCP); 
    int ret = srslte_netsource_init(&wishful_command_server, "0.0.0.0", portNo, SRSLTE_NETSOURCE_TCP); 
    
    if(ret != 0)
    {
        perror("failed to initialize socket");
        exit(-1);
    }    
    
    pipe_producer_t* p_command = pipe_producer_new(command_pipe);
    pipe_free(command_pipe);
            
    while(true)
    {
        char buf[120];
        printf("[srslte] : parameter reception thread ready \n");
        int serv_state = srslte_netsource_read(&wishful_command_server, buf, 120);
        if(serv_state <  0)
        {
            fprintf(stderr, "Error receiving from network\n");
            exit(-1);
        }
        else if(serv_state == 0)
        {
            //connection closed
        }
        else
        {
            JSON_Value *schema = json_parse_string(buf);
            bool wants_metric = json_object_get_number(json_object(schema), "wants_metric");
            int which_metric = json_object_get_number(json_object(schema), "which_metric");
            bool make_config = json_object_get_number(json_object(schema), "make_config");
            int which_config = json_object_get_number(json_object(schema), "which_config");
            int config_value = json_object_get_number(json_object(schema), "config_value");
            json_value_free(schema);
            //printf("wants metric is %d\n",num);
            
            
             
            wishful_command comm;
            comm.which_config = which_config;
            comm.config_value = config_value;
            comm.make_config  = (make_config == 1)?true:false;
            comm.wants_metric = (wants_metric == 1)?true:false;
            comm.which_metric = which_metric;
            
            if(comm.wants_metric)
            {
                printf("[SRSLTE] Wishful has requested metric number : %d \n", comm.which_metric);
            }
            else
            {
               
            }
            pipe_push(p_command,(void*)&comm, 1);
        }
    }
    
}

void init_wishful_receive(pipe_t* command_pipe)
{
    if(sem_init(&wishful_sem,0,0))
    {
        perror("wishful sem init");
        exit(-1);
    }
    pthread_attr_t attr;
    struct sched_param param;
    param.sched_priority = 0;  
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &param);
      if (pthread_create(&wishful_thread_receive, NULL, wishful_thread_receive_run,(void*)command_pipe)) {
    perror("pthread_create wishful failed");
    exit(-1);
  }  
    
}
void * wishful_thread_send_run(void *args)
{
    
    pipe_t* command_pipe = args;
    
    int portNo = 2222;
    int ret =  srslte_netsink_init(&wishful_metric_client, "0.0.0.0", portNo, SRSLTE_NETSINK_TCP);
    srslte_filesink_init(&CSI_sink, prog_args.csi_output_file, SRSLTE_COMPLEX_FLOAT_BIN);
    if(ret != 0)
    {
        perror("failed to connect to server");
        exit(-1);
    }
    
    
    pipe_consumer_t* c = pipe_consumer_new(command_pipe);
    char data[5];
    strncpy(data,"hello", 5);
    if(srslte_netsink_write(&wishful_metric_client, data, 5) < 0)
    {
        fprintf(stderr, "Error sending data through TCP socket\n");
    }

    while(true)
    {
      wishful_command rece[1];
      printf("[srslte] : metric tx thread ready \n");
      pipe_pop(c,rece,1);
      //printf("buffer contains :%4.1f\n", rece[0].config_value);
      wishful_response wish;
      JSON_Value *root_value = json_value_init_object();
      JSON_Object *root_object = json_value_get_object(root_value);
      if(rece[0].wants_metric)
      {
        wish.which_metric = 0;
        wish.metric_value = 0;
        wish.is_reconfig = false;
        switch(rece[0].which_metric)
        {
          case CFO:
          wish.which_metric = CFO;
          wish.metric_value = srslte_ue_sync_get_cfo(&ue_sync)/1000;
          break;
          case SNR:
          wish.which_metric = SNR;
          wish.metric_value = 10*log10(phy_metrics.rsrp/phy_metrics.noise);
          break;
          case RSRP:
          wish.which_metric = RSRP;
          wish.metric_value = phy_metrics.rsrp;
          break;
          case RSRQ:
          wish.which_metric = RSRQ;
          wish.metric_value = phy_metrics.rsrq;
          break;
          case NOISE: 
          wish.which_metric = NOISE;
          wish.metric_value = phy_metrics.noise;
          break;
          case CSI:
          wish.which_metric = CSI;
          printf("writing CSI to  %s \n",prog_args.csi_output_file);
          srslte_filesink_write(&CSI_sink, ue_dl.ce[0], 12*ue_dl.cell.nof_prb);
          wish.metric_value = true;
          break;
          case N_FRAMES:
          wish.which_metric = N_FRAMES;
          wish.metric_value = phy_metrics.nframes;
          break;
          case PDSCH_MISS:
          wish.which_metric = PDSCH_MISS;
          wish.metric_value = phy_metrics.pdsch_miss;
          break;
          case PDCCH_MISS:
          wish.which_metric = PDCCH_MISS;
          wish.metric_value = phy_metrics.pdcch_miss;
          break;
          case MOD:
          wish.which_metric = MOD;
          wish.metric_value = ue_dl.pdsch_cfg.grant.mcs.idx;
          break;
          case TBS:
          wish.which_metric = TBS;
          wish.metric_value = ue_dl.pdsch_cfg.grant.mcs.tbs;
          break;
          default:
          break;
        }
        wish.reconfig_value = -1;
        wish.which_reconfig = -1;

            
        } else {
          wish.which_metric = 0;
          wish.metric_value = 0;

          wish.reconfig_value  = rece[0].config_value;
          wish.which_reconfig = rece[0].which_config;
          wish.is_reconfig = true;
          printf("[SRSLTE] about to reconfigure \n");
          switch(wish.which_reconfig)
          {
            case GAIN:
              printf("Set RX gain: %.1f dB\n", srslte_rf_set_rx_gain(&rf, wish.reconfig_value));
            break;
            case FREQ:
              printf("Set RX Freq to %.1f Hz\n", srslte_rf_set_rx_freq(&rf, wish.reconfig_value));
            break;
            default:
            break;
          }
        }
        char *serialized_string = NULL;
        json_object_set_number(root_object, "which_metric", wish.which_metric);
        json_object_set_number(root_object, "metric_value", wish.metric_value);
        json_object_set_number(root_object, "is_reconfig", wish.is_reconfig);
        json_object_set_number(root_object, "which_reconfig", wish.which_reconfig);
        json_object_set_number(root_object, "reconfig_value", wish.reconfig_value);
        serialized_string = json_serialize_to_string_pretty(root_value);
        //puts(serialized_string);
        int size = strlen(serialized_string);
        //printf("size of string is %d\n", size);
        if(srslte_netsink_write(&wishful_metric_client, serialized_string, size) < 0)
        {
            fprintf(stderr, "Error sending data through TCP socket\n");
        }
    }
    
    srslte_filesink_free(&CSI_sink);
    

}

void  init_wishful_send(pipe_t* command_pipe)
{
    
    pthread_attr_t attr;
    struct sched_param param;
    param.sched_priority = 0;  
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &param);
      if (pthread_create(&wishful_thread_send, NULL, wishful_thread_send_run,(void*)command_pipe)) {
    perror("pthread_create wishful failed");
    exit(-1);
  }
    
}

#endif
