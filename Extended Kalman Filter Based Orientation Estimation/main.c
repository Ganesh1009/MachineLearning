/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "app_timer.h"
#include "fds.h"
#include "app_error.h"
#include "app_util.h"

#include "nrf_cli.h"
#include "nrf_cli_rtt.h"
#include "nrf_cli_types.h"

#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_flash.h"
#include "nrf_fstorage_nvmc.h"

#include "nrf_mpu.h"
#include "nrf_stack_guard.h"

#include "nrf_drv_spi.h"

#include "nrf_cli_cdc_acm.h"
#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"

// DFKI BSN INCLUDES !!!!
#include "bsn_imu_data.h"
#include "bsn_imu_calib.h"
#include "bsn_imu_orientation.h"
#include "matrix.h"



#define TIMESTAMP 1000
#define QUAT_DIM 4
#define NORM_DIM 3
#define GRAVITY_CONSTANT 9.8

/* If enabled then CYCCNT (high resolution) timestamp is used for the logger. */
#define USE_CYCCNT_TIMESTAMP_FOR_LOG 0

#if NRF_LOG_BACKEND_FLASHLOG_ENABLED
NRF_LOG_BACKEND_FLASHLOG_DEF(m_flash_log_backend);
#endif

#if NRF_LOG_BACKEND_CRASHLOG_ENABLED
NRF_LOG_BACKEND_CRASHLOG_DEF(m_crash_log_backend);
#endif

/* Counter timer. */
APP_TIMER_DEF(m_timer_0);
volatile bool sampling_flag;

#define CS_PIN	29
#define  eepromFileSIZE 256
#define SPI_INSTANCE  0 						/**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static uint8_t       m_rx_buf[22];    /**< RX buffer. */
static uint8_t       eeprom_buf[255];    			/**< RX buffer. */
static uint8_t       m_rx_buf_right[22*7];    			/**< RX buffer. */

static uint8_t  eeprom_static_buf[eepromFileSIZE]= {
    /* Green Hub */
    0x4d, 0x53, 0x30, 0x36, 0x30, 0x36, 0x31, 0x38, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x25, 0x06, 0x81, 0x40, 0xa9, 0x7f, 0xff, 0xc0, 0x85, 0xeb, 0x71, 0xbf, 0x4d, 0xa9, 0xa6, 0x40, 0x75, 0x93, 0xe4, 0x40, 0x74, 0xda, 0x50, 0x3f, 0x00, 0x20, 0xff, 0x44, 0x00, 0x90, 0xf7, 0x44, 0x00, 0x10, 0x00, 0x45, 0x32, 0x56, 0x9c, 0x3b, 0x47, 0x07, 0x12, 0x39, 0x15, 0xd0, 0x53, 0xb9, 0x07, 0xfc, 0xcd, 0xb8, 0x82, 0x71, 0x9c, 0x3b, 0x94, 0xf2, 0x98, 0xb8, 0x2b, 0x96, 0x48, 0x39, 0x3c, 0x85, 0xea, 0x37, 0x76, 0xb5, 0x9c, 0x3b, 0xcf, 0xec, 0x8b, 0x3a, 0x22, 0xbf, 0xbf, 0x37, 0x15, 0x7a, 0x6c, 0xb8, 0x7b, 0xf8, 0xbf, 0xb7, 0xad, 0x38, 0x8c, 0x3a, 0x7d, 0xbe, 0x8f, 0xb7, 0x20, 0x5f, 0x2c, 0x38, 0x56, 0x7b, 0x83, 0x36, 0xbb, 0xfa, 0x8b, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x6f, 0x12, 0x83, 0x3c, 0x00, 0x00, 0x00, 0x00, 0xb4, 0xc2, 0x66, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf1, 0xf0, 0x70, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0x81, 0x8a, 0x27
};


/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
    case APP_USBD_EVT_STOPPED:
        app_usbd_disable();
        break;
    case APP_USBD_EVT_POWER_DETECTED:
        if (!nrf_drv_usbd_is_enabled())
        {
            app_usbd_enable();
        }
        break;
    case APP_USBD_EVT_POWER_REMOVED:
        app_usbd_stop();
        break;
    case APP_USBD_EVT_POWER_READY:
        app_usbd_start();
        break;
    default:
        break;
    }
}

/**
 * @brief Command line interface instance
 * */
#define CLI_EXAMPLE_LOG_QUEUE_SIZE  (4)
NRF_CLI_CDC_ACM_DEF(m_cli_cdc_acm_transport);
NRF_CLI_DEF(m_cli_cdc_acm,
            "\n",//"usb_cli:~$ ",
            &m_cli_cdc_acm_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);


static void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    sampling_flag = true;
}

static void cli_start(void)
{
    ret_code_t ret;
    ret = nrf_cli_start(&m_cli_cdc_acm);
    APP_ERROR_CHECK(ret);

}

static void cli_init(void)
{
    ret_code_t ret;
    ret = nrf_cli_init(&m_cli_cdc_acm, NULL, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
}

static void usbd_init(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_handler = app_usbd_event_execute,
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm =
        app_usbd_cdc_acm_class_inst_get(&nrf_cli_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");
        app_usbd_enable();
        app_usbd_start();
    }

    /* Give some time for the host to enumerate and connect to the USB CDC port */
    nrf_delay_ms(1000);
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context)
{
    spi_xfer_done = true;
}

void spim_config(void) {
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = 13;
    spi_config.miso_pin = 02;
    spi_config.mosi_pin = 17;
    spi_config.sck_pin  = NRF_GPIO_PIN_MAP(1,15);
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M ;
    spi_config.mode = NRF_DRV_SPI_MODE_3;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

int main(void) {
    ret_code_t ret;
    nrf_gpio_cfg_output(LED_3);
    nrf_gpio_pin_clear(LED_3);
    nrf_gpio_cfg_output(15);
    nrf_gpio_pin_clear(15);

    nrf_gpio_cfg_output(CS_PIN);
    nrf_gpio_pin_set(CS_PIN);
    nrf_delay_ms(1000);
    nrf_gpio_pin_clear(CS_PIN);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    ret = app_timer_create(&m_timer_0, APP_TIMER_MODE_REPEATED, timer_handle);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(m_timer_0, APP_TIMER_TICKS(10), NULL);
    APP_ERROR_CHECK(ret);

    cli_init();
    usbd_init();
    cli_start();
    spim_config();

    uint8_t nrRightNodes = 0;
    int8_t DetectTXLength = 7*3; 	//every node will send 3 bytes
    uint8_t  NodeArray[7];

    nrf_delay_ms(3000);
    nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\n Dongle-Hub");

    //Sync phase
    nrf_cli_fprintf(&m_cli_cdc_acm, 2, "\n\nStart");		//2 corresponds to red color
    nrf_gpio_pin_clear(CS_PIN);
    nrf_delay_us(100);
    nrf_gpio_pin_set(CS_PIN);
    nrf_delay_us(1000);
    nrRightNodes = 0;
    for(int i=0; i<7; i++) {
        nrf_gpio_pin_clear(CS_PIN);
        nrf_delay_us(20);
        spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf+(i*3), 3));
        while(!spi_xfer_done) {}
        nrf_gpio_pin_set(CS_PIN);
        nrf_delay_us(30);
    }

    for(int i= 0; i< DetectTXLength; i=i+3) {
        if((m_rx_buf[i] == 0xFA) &&  (m_rx_buf[i+2] == 0xFB)) {
            NodeArray[i/3] = m_rx_buf[i+1];
            nrRightNodes++;
        }
        nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nRight: 0x%2X 0x%2X 0x%2X",m_rx_buf[i],m_rx_buf[i+1],m_rx_buf[i+2]);
        nrf_delay_ms(20);
    }

    nrf_delay_ms(50);
    memset(m_rx_buf, 0, 22);
    nrf_delay_ms(1);

    // read eeprom
    for(int i=0; i< 7 ; i++) {            // Only read the eeprom data of the nodes that are actually available

        nrf_gpio_pin_clear(CS_PIN);
        nrf_delay_us(20);
        spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &nrRightNodes, 1, eeprom_buf, 255));
        while(!spi_xfer_done) {}
        nrf_gpio_pin_set(CS_PIN);

        if(i<nrRightNodes) {		// just print those that
            nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nRight eepromData: Node %d\n",NodeArray[i] );
            for(int j = 0; j<55 ; j++) {
                nrf_cli_fprintf(&m_cli_cdc_acm, 2,"%02X ",eeprom_buf[j] );
            }
            if(memcmp(eeprom_buf+1, eeprom_static_buf+1, sizeof(eeprom_buf)-1) !=0) {
                nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nError on Eeprom Node: %d\n",NodeArray[i] );
            } else {
                nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nEeprom in Node: %d ->OK\n",NodeArray[i] );
            }
        }
        nrf_delay_ms(50);
    }
    nrf_delay_ms(500);
    nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nNr of Nodes: Right:%d\n", nrRightNodes);

    imu_eeprom_layout_v00_t* eeprom_data = (imu_eeprom_layout_v00_t*) &eeprom_static_buf;
    imu_t imudata = {0};
    imu_calibrated_t imu_calibrated = {0};
    imu_orientation_t imu_orientation = {0};

    int TimeHub =0, to_time_hub = 0, from_time_hub = 0;

    sampling_flag = false;
    nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nStarted calibration matrix initialization.");
    calibration_matrix_initialization(eeprom_data);
    nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nCalibration matrix initialization successful.");
    bsn_imu_orientation_initialize();
    while( nrRightNodes !=0) {
        while(!sampling_flag) {}
        sampling_flag = false;
        if(nrRightNodes>0) {
            for(int i = 0; i<nrRightNodes; i++) {		// Do individual tranmssions for each node
                spi_xfer_done = false;
                nrf_delay_us(100);
                nrf_gpio_pin_clear(CS_PIN);
                nrf_delay_us(5);			// Spare time to give the node masters enough time to start their transmissions
                APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf_right+(i*22), 22));	// Maybe here the TX needs to be one by one!!!!!
                while(!spi_xfer_done) {}
                nrf_gpio_pin_set(CS_PIN);
            }
        }
        TimeHub = app_timer_cnt_get() * 1000 / 16384;

        // nrf_cli_fprintf(&m_cli_cdc_acm, 2, "\n\r %10d - ", TimeHub);

        imudata.time = TimeHub;
        imudata.acc[0] = m_rx_buf_right[2] << 8 | m_rx_buf_right[3];
        imudata.acc[1] = m_rx_buf_right[4] << 8 | m_rx_buf_right[5];
        imudata.acc[2] = m_rx_buf_right[6] << 8 | m_rx_buf_right[7];
        imudata.gyr[0] = m_rx_buf_right[10] << 8 | m_rx_buf_right[11];
        imudata.gyr[1] = m_rx_buf_right[12] << 8 | m_rx_buf_right[13];
        imudata.gyr[2] = m_rx_buf_right[14] << 8 | m_rx_buf_right[15];
        imudata.mag[0] = m_rx_buf_right[16] << 8 | m_rx_buf_right[17];
        imudata.mag[1] = m_rx_buf_right[18] << 8 | m_rx_buf_right[19];
        imudata.mag[2] = m_rx_buf_right[20] << 8 | m_rx_buf_right[21];

        //now do the processing of the data

        //1. apply calibration
        from_time_hub = app_timer_cnt_get()*1000000 / 16384;
        nrf_gpio_pin_set(15);
        imu_calibrated = bsn_imu_calib_applycalibration(eeprom_data, &imudata);
        nrf_gpio_pin_clear(15);
        // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"GB=%d,%d,%d|AB=%d,%d,%d|MB=%d,%d,%d",
        // (int)eeprom_data->bg[0]*1000,(int)eeprom_data->bg[1]*1000,(int)eeprom_data->bg[2]*1000,
        // (int)eeprom_data->ba[0]*1000,(int)eeprom_data->ba[1]*1000,(int)eeprom_data->ba[2]*1000,
        // (int)eeprom_data->bm[0]*1000,(int)eeprom_data->bm[1]*1000,(int)eeprom_data->bm[2]*1000);
        // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"GD=%d,%d,%d,%d,%d,%d,%d,%d,%d|AD=%d,%d,%d,%d,%d,%d,%d,%d,%d|MD=%d,%d,%d,%d,%d,%d,%d,%d,%d",
        // (int)(eeprom_data->krginv[0]*1000000),(int)(eeprom_data->krginv[1]*1000000),(int)(eeprom_data->krginv[2]*1000000),(int)(eeprom_data->krginv[3]*1000000),(int)(eeprom_data->krginv[4]*1000000),(int)(eeprom_data->krginv[5]*1000000),(int)(eeprom_data->krginv[6]*1000000),(int)(eeprom_data->krginv[7]*1000000),(int)(eeprom_data->krginv[8]*1000000),
        // (int)(eeprom_data->krainv[0]*1000000),(int)(eeprom_data->krainv[1]*1000000),(int)(eeprom_data->krainv[2]*1000000),(int)(eeprom_data->krainv[3]*1000000),(int)(eeprom_data->krainv[4]*1000000),(int)(eeprom_data->krainv[5]*1000000),(int)(eeprom_data->krainv[6]*1000000),(int)(eeprom_data->krainv[7]*1000000),(int)(eeprom_data->krainv[8]*1000000),
        // (int)(eeprom_data->krminv[0]*1000000),(int)(eeprom_data->krminv[1]*1000000),(int)(eeprom_data->krminv[2]*1000000),(int)(eeprom_data->krminv[3]*1000000),(int)(eeprom_data->krminv[4]*1000000),(int)(eeprom_data->krminv[5]*1000000),(int)(eeprom_data->krminv[6]*1000000),(int)(eeprom_data->krminv[7]*1000000),(int)(eeprom_data->krminv[8]*1000000)
      // );


        to_time_hub = app_timer_cnt_get()*1000000 / 16384;
        int calib_time = (to_time_hub-from_time_hub);
        //2. pass on to the filter
        from_time_hub = app_timer_cnt_get()*1000000 / 16384;
        nrf_gpio_pin_set(15);
        bsn_imu_orientation_add_sample(&imu_calibrated);
        nrf_gpio_pin_clear(15);
        to_time_hub = app_timer_cnt_get()*1000000 / 16384;
        int orint_time = (to_time_hub - from_time_hub);
        //3. request updated orienation
        imu_orientation = bsn_imu_orientation_get_quaternion();
        nrf_cli_fprintf(&m_cli_cdc_acm, 2,"{\"CT\":%d,\"OT\":%d,\"TOT\":%d,\"a\":[%d,%d,%d],\"g\":[%d,%d,%d],\"m\":[%d,%d,%d],\"q\":[%d,%d,%d,%d]}\n",
          calib_time,orint_time,TimeHub,(int)(imu_calibrated.acc[0]*1000),
          (int)(imu_calibrated.acc[1]*1000), (int)(imu_calibrated.acc[2]*1000),
          (int)(imu_calibrated.gyr[0]*1000), (int)(imu_calibrated.gyr[1]*1000),
          (int)(imu_calibrated.gyr[2]*1000), (int)(imu_calibrated.mag[0]*1000),
          (int)(imu_calibrated.mag[1]*1000), (int)(imu_calibrated.mag[2]*1000),
          (int)(imu_orientation.w*1000),(int)(imu_orientation.x*1000),
          (int)(imu_orientation.y*1000),(int)(imu_orientation.z*1000));
    }
    bsn_imu_orientation_free();
}
