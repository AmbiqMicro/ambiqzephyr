//*****************************************************************************
//
//! @file am_vos_thf.c
//!
//! @brief Sensory THF engine control
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include "SensoryLib.h"
#include "am_vos_thf.h"

LOG_MODULE_REGISTER(am_vos_thf);

//    #include "thfft_alexa_enus_v3c_dsp_64kb_pc40\thfft_alexa_enus_v3c_dsp_64kb_search_2_pc40.c"
#include "thfft_alexa_enus_v3c_dsp_64kb_search_4_pc40.c"
#include "thfft_alexa_enus_v3c_dsp_64kb_am_pc40.c"

#define CURRENT_SDET            SDET_NONE
#define configUSE_THF_Static_Alloc  1

#define TRIGGER_DELAY 	        240 	        // override trigger grammar delay value to find end of trigger
                                                // with more accuracy

#define COMMAND_DELAY           0   	        // command delay: use grammar value
#define COMMAND_TIMEOUT         3 		// 5 seconds after trigger to say the command

#if configUSE_THF_Logging
#define SENSORY_LOG_ITEMS       200
#endif // configUSE_THF_Logging

#if configUSE_THF_Static_Alloc
    #define THF_STATIC_MEM_SIZE     24 * 1024
#endif // configUSE_THF_Static_Alloc


//***********************************************************************
// Global Sensory Instance
//***********************************************************************
typedef struct _VosThfInfo {
    appStruct_T         sAppStruct;
    uint32_t            ui32DetectCount;

#if AUDIO_BUFFER_LEN > 0
    SAMPLE              pin16AudioBuffer[AUDIO_BUFFER_LEN];
#endif

#if configUSE_THF_Logging
    SENSORY_LOG_OBJ     psSensoryLog[SENSORY_LOG_ITEMS];
#endif // configUSE_THF_Logging

#if configUSE_THF_Static_Alloc
    uint8_t             pui8ThfHeap[THF_STATIC_MEM_SIZE];
#endif // configUSE_THF_Static_Alloc
} VosThfInfo;

VosThfInfo g_sVosThf = {
    .ui32DetectCount = 0,

#if configUSE_THF_Static_Alloc
    .pui8ThfHeap = {0,},
#endif // configUSE_THF_Static_Alloc
};

// configure t2siStruct with recognition parameters
// allocate memory for Sensory Persistent structure
// initialize audio buffer parameters
// initialized LPSD
// initialize recognizer via call to SensoryProcessInit

errors_t SensoryInitialize(void)
{
    unsigned int ww_spp_size = 0; // size of spp
    unsigned int cmd_spp_size = 0;

    errors_t result;

    appStruct_T *ap = &(g_sVosThf.sAppStruct);
    t2siStruct *t = &ap->_t;

    // Set up recognition control parameters in the t2siStruct.
    // NOTE: t->net, gram, maxResults and maxTokens affect the amount of memory needed
    // (which will be calculated by the call to SensoryAlloc).

    // Here we zero out appStruct including embedded t2siStruct.
    // Warning: if you do this after the persistent structure is malloc'd you will
    // overwrite the pointer without freeing the structure.

    memset(ap, 0, sizeof(*ap));

    t->maxResults = MAX_RESULTS;
    t->maxTokens  = MAX_TOKENS;

    t->sdet_type = CURRENT_SDET; // SDET_NONE or SDET_LPSD
    t->knob = T2SI_DEFAULT_KNOB;
    t->paramAOffset = 0;
    t->enableLogging = 0;   // default is enabling logging

#if configUSE_THF_Logging
    if (t->enableLogging)
    {
        SensoryLogInit(ap, g_sVosThf.psSensoryLog, SENSORY_LOG_ITEMS);
        SensoryLog(t, SENSORY_LOG_RESET,0,0);
    }
#endif // configUSE_THF_Logging

#if AUDIO_BUFFER_LEN > 0
    // initialize audio buffer items. Do this before calling SensoryLPSDInit.
    ap->audioBufferLen = AUDIO_BUFFER_LEN;
    // if not using malloc for the audio buffer, just point audioBufferStart to the statically
    // allocated audio buffer.

    ap->audioBufferStart = g_sVosThf.pin16AudioBuffer;
    ap->audioPutIndex = ap->audioGetIndex = ap->lpsdGetIndex = 0;
    ap->audioGetFrameCounter = 0;
    ap->audioFilledCount = 0;

    // Initialize Sound Detector (OK to do this even for SDET_NONE)
    // Do this after setting up the audio buffer items in appStruct.  You will want to
    // call SensoryProcessInit after this, before the next call to SensoryProcessData
//        SensoryLPSDInit(ap);
#endif

    t->delay = TRIGGER_DELAY;
    t->timeout = 0;

    t->net  = (u32)dnn_wakeword_netLabel;
    t->gram = (u32)gs_wakeword_grammarLabel;

    SensoryAlloc (&(g_sVosThf.sAppStruct), &ww_spp_size);               // Note: need to call SensoryAlloc() to set one library internal variable with calculated size whenever different net and gram are used
    LOG_INF("[AM-VoS] THF spp ww_spp_size = %d bytes\n", ww_spp_size);

    // allocate the persistent structure memory
    if(ww_spp_size > THF_STATIC_MEM_SIZE) {
        LOG_INF("[AM-VoS] THF heap memory is not enough!!\n", ww_spp_size);
        return ERR_NOT_OK;
    }
    t->spp = g_sVosThf.pui8ThfHeap;
    
    result = SensoryProcessInit(ap);
    if (result)
    {
        return ERR_NOT_OK;   // error!
    }

    return 0;
}

// free the Sensory persistent structure
errors_t SensoryTerminate()
{
    appStruct_T *ap = &(g_sVosThf.sAppStruct);
    t2siStruct *t = &ap->_t;

#if configUSE_THF_Static_Alloc
    t->spp = NULL;
#else // configUSE_THF_Static_Alloc
    if (t->spp) {
        vPortFree(t->spp);
        t->spp = NULL;
    }
#endif // configUSE_THF_Static_Alloc
    return 0;
}

uint8_t am_vos_engine_init(void)
{
    if(SensoryInitialize())  // load alexa net and grammar as a default
    {
        LOG_INF("Sensory THF init fail!!\n");
        return 1;
    }

    return 0;
}


// A new brick of samples is available to be processed through the recognizer
// This function returns the result from SensoryProcessData().
//extern appStruct_T appStruct; 
static errors_t SensoryProcessBrick(s16 *brick, appStruct_T *ap, bool bFaEnabled)
{
    errors_t result;
    //appStruct_T *ap = &appStruct;
    t2siStruct *t = &ap->_t;

    result = SensoryProcessData(brick, ap);

    // optionally print a warning message if the recognizer ran out of token memory.
    // if this happens you need to increase the MAX_TOKENS parameter in SensoryLib.h.
    if (t->outOfMemory) {
        //am_util_stdio_printf("Error: Out of Token Memory\n");
        t->outOfMemory = 0;
    }

    if (result == ERR_OK) {
        if (t->wordID && ((!t->nnpqScore && !t->nnpqThreshold && !t->nnpqPass) || ((t->nnpqScore>0) && t->nnpqPass)))	{ // non-nnpq mode or nnpq mode passing score check
            //am_util_stdio_printf("Recognition***** wordID = %d, score = %d\n", t->wordID, t->finalScore);
            //if (t->svScore >= 0) am_util_stdio_printf("SV score = %ld\n", t->svScore);
            //if (t->nnpqPass) am_util_stdio_printf("NNPQ score= %d, NNPQ threshold= %d, NNPQ check pass= %d\n", t->nnpqScore, t->nnpqThreshold, t->nnpqPass);

            // optional: find end-point index in audio buffer where recognition was decided (epIndex)
            // and also the number of samples in the audio buffer after that point (tailCount)
            // which should correspond to the latency between the end of the trigger phrase and the recognition
            // decision.
#if configUSE_THF_Logging
            if (t->enableLogging) SensoryPrintLog(ap);
#endif
            //result = ERR_OK;
        }
        else {
            if(bFaEnabled)
            {
                result = ERR_RECOG_FAIL;
                //am_util_stdio_printf("Not recognized\n");
            }
        }
    
        SensoryProcessRestart(ap);  // restart recognizer
    }
    else if (result != ERR_NOT_FINISHED) {
        //am_util_stdio_printf("SensoryProcessData error 0x%lx, wordID= %d\n", result, t->wordID);
        SensoryProcessInit(ap);  // reset recognizer
    }
    return result;
}

void am_vos_engine_process(int16_t *frame, int16_t i16InputLength, uint8_t *pui8KwdDetectedFlag)
{
    errors_t result = ERR_NOT_FINISHED;
    appStruct_T *ap = &(g_sVosThf.sAppStruct);

    t2siStruct *t = &ap->_t;
    unsigned int size;

    result = SensoryProcessBrick(frame, ap, false);
    if(result == ERR_OK)
    {
        *pui8KwdDetectedFlag = 1;

        LOG_INF("\n[AM-VoS] Keyword Detected! [%d] [%d]", t->wordID, ++g_sVosThf.ui32DetectCount);
    }
}
