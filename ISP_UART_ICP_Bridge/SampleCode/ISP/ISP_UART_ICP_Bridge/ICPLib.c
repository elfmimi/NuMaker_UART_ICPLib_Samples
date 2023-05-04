#include <stdio.h>
#include "NuMicro.h"
#include "ICPLib.h"


#define ICP_LIB_VERSION             0x0010

#define ICP_ENTRY_CODE              0x55AA03

#define FMC_ISPCMD_MASS_ERASE       0x26    /* Mass Erase */


#define ICP_PIN_RST                 PF5
#define ICP_PIN_CLK                 PA10
#define ICP_PIN_DAT                 PA9

#define ICP_PIN_LV0                 PA6
#define ICP_PIN_LV1                 PA7

#define ICP_CLOCK_CYCLE()               \
do                                      \
{                                       \
    CLK_SysTickDelay(1);                \
    ICP_PIN_CLK = 1;                    \
    CLK_SysTickDelay(1);                \
    ICP_PIN_CLK = 0;                    \
} while(0)

#define ICP_READ_BIT(bit)               \
do                                      \
{                                       \
    CLK_SysTickDelay(1);                \
    bit = ICP_PIN_DAT;                  \
    ICP_PIN_CLK = 1;                    \
    CLK_SysTickDelay(1);                \
    ICP_PIN_CLK = 0;                    \
} while(0) 

#define ICP_WRITE_BIT(bit)              \
do                                      \
{                                       \
    ICP_PIN_DAT = bit;                  \
    ICP_CLOCK_CYCLE();                  \
} while(0) 


static uint32_t g_u32ResetActiveLevel = 0;  // 0: Low, 1: High

static uint32_t g_u32IOMode_RST = 0;
static uint32_t g_u32IOMode_CLK = 0;
static uint32_t g_u32IOMode_DAT = 0;

static uint32_t g_u32IOMode_LV0 = 0;
static uint32_t g_u32IOMode_LV1 = 0;

static uint32_t g_u32IOLevel_RST = 1;
static uint32_t g_u32IOLevel_CLK = 1;
static uint32_t g_u32IOLevel_DAT = 1;

static uint32_t g_u32IOLevel_LV0 = 1;
static uint32_t g_u32IOLevel_LV1 = 1;

static uint32_t g_u32MFP_RST = 0;
static uint32_t g_u32MFP_CLK = 0;
static uint32_t g_u32MFP_DAT = 0;

static uint32_t g_u32MFP_LV0 = 0;
static uint32_t g_u32MFP_LV1 = 0;


typedef struct
{
    uint32_t u32MassEraseDelayTime;
    uint32_t u32MassEraseHoldTime;
    uint32_t u32PageEraseDelayTime;
    uint32_t u32PageEraseHoldTime;
    uint32_t u32ProgramDelayTime;
    uint32_t u32ProgramHoldTime;
    uint32_t u32ReadDelayTime;
    uint32_t u32ReadHoldTime;
} STR_ICP_TIMING;


static STR_ICP_TIMING g_ICPTiming = {320000, 120, 120000, 120, 60, 10, 1, 5};

static uint8_t CRC8(uint8_t u8Data, uint8_t u8CRC_Value)
{
    static const uint8_t u8CRC_Table[16] =
    {
        0x00, 0x07, 0x0E, 0x09,
        0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31,
        0x24, 0x23, 0x2A, 0x2D
    };

    u8CRC_Value = (u8CRC_Value << 4) ^ u8CRC_Table[((u8CRC_Value) ^ u8Data) >> 4];
    u8CRC_Value = (u8CRC_Value << 4) ^ u8CRC_Table[((u8CRC_Value >> 4) ^ u8Data) & 0x0F];

    return u8CRC_Value;
}

static uint32_t GetCRCValue(uint32_t *u32Buffer, uint32_t u32Count)
{
    uint8_t u8CRC_Value[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t *pu8Buf;
    uint32_t i;

    pu8Buf = (uint8_t *)u32Buffer;

    for (i = 0; i < u32Count; i++)
    {
        u8CRC_Value[0] = CRC8(pu8Buf[i * 4 + 0], u8CRC_Value[0]);
        u8CRC_Value[1] = CRC8(pu8Buf[i * 4 + 1], u8CRC_Value[1]);
        u8CRC_Value[2] = CRC8(pu8Buf[i * 4 + 2], u8CRC_Value[2]);
        u8CRC_Value[3] = CRC8(pu8Buf[i * 4 + 3], u8CRC_Value[3]);
    }

    return ((u8CRC_Value[3] << 24) | (u8CRC_Value[2] << 16) | (u8CRC_Value[1] << 8) | u8CRC_Value[0]);
}

static __INLINE void ICP_SetGPIOMode(GPIO_T *port, uint32_t u32Pin, uint32_t u32Mode)
{
    port->MODE = (port->MODE & ~(0x3 << (u32Pin << 1))) | (u32Mode << (u32Pin << 1));
}

static void ICP_SetReset(uint32_t bActive)
{
    uint32_t u32Level = (g_u32ResetActiveLevel ^ bActive)? 0 : 1;

    ICP_PIN_RST = u32Level;

    while(ICP_PIN_RST != u32Level);
}

static void ICP_WriteBits(uint32_t u32Data, uint32_t u32Bits)
{
    uint32_t i, u32BitData;

    for (i = 0; i < u32Bits; i++)
    {
        u32BitData = (u32Data >> (u32Bits - i - 1)) & 0x01;

        ICP_WRITE_BIT(u32BitData);
    }
}

static uint32_t ICP_ReadBits(uint32_t u32Bits)
{
    uint32_t i, u32Data = 0, u32BitData;

    for (i = 0; i < u32Bits; i++)
    {
        ICP_READ_BIT(u32BitData);

        u32Data |= (u32BitData << (u32Bits - i - 1));
    }

    return u32Data;
}

static void ICP_SendCommand(uint32_t u32Cmd, uint32_t u32Addr)
{
    ICP_WriteBits(((u32Addr << 8) | u32Cmd), 32);
}

static uint32_t ICP_ReadData(uint32_t u32nCont)
{
    uint32_t u32Data;

    ICP_SetGPIOMode(PA, 9, GPIO_MODE_INPUT);

    ICP_CLOCK_CYCLE();

    CLK_SysTickDelay(1);

    u32Data = ICP_ReadBits(32);

    ICP_SetGPIOMode(PA, 9, GPIO_MODE_OUTPUT);

    ICP_CLOCK_CYCLE();

    ICP_WriteBits(u32nCont, 1);

    CLK_SysTickDelay(5);

    return u32Data;
}

static void ICP_WriteData(uint32_t u32Data, uint32_t u32nCont, uint32_t u32DelayTime, uint32_t u32HoldTime)
{
    uint32_t u32Delay;

    ICP_CLOCK_CYCLE();

    CLK_SysTickDelay(1);

    ICP_WriteBits(u32Data, 32);

    ICP_CLOCK_CYCLE();

    while (u32DelayTime)
    {
        u32Delay = u32DelayTime;

        if (u32Delay > 80000)
            u32Delay = 80000;

        CLK_SysTickDelay(u32Delay);

        u32DelayTime -= u32Delay;
    }

    ICP_WriteBits(u32nCont, 1);

    CLK_SysTickDelay(u32HoldTime);
}




/*--------------------------*/
/*      ICP Command         */
/*--------------------------*/
uint32_t ICP_GetVersion(void)
{
    return ICP_LIB_VERSION;
}

void ICP_Init(uint32_t u32ResetActiveLevel)
{
    g_u32ResetActiveLevel = u32ResetActiveLevel;

    g_u32IOMode_RST = (PF->MODE >> ( 5 << 1)) & 0x03;
    g_u32IOMode_CLK = (PA->MODE >> (10 << 1)) & 0x03;
    g_u32IOMode_DAT = (PA->MODE >> ( 9 << 1)) & 0x03;

    g_u32IOMode_LV0 = (PA->MODE >> ( 6 << 1)) & 0x03;
    g_u32IOMode_LV1 = (PA->MODE >> ( 7 << 1)) & 0x03;

    g_u32IOLevel_RST = ICP_PIN_RST;
    g_u32IOLevel_CLK = ICP_PIN_CLK;
    g_u32IOLevel_DAT = ICP_PIN_DAT;

    g_u32IOLevel_LV0 = ICP_PIN_LV0;
    g_u32IOLevel_LV1 = ICP_PIN_LV1;

    g_u32MFP_RST = SYS->GPF_MFPL & SYS_GPF_MFPL_PF5MFP_Msk;
    g_u32MFP_CLK = SYS->GPA_MFPH & SYS_GPA_MFPH_PA10MFP_Msk;
    g_u32MFP_DAT = SYS->GPA_MFPH & SYS_GPA_MFPH_PA9MFP_Msk;

    g_u32MFP_LV0 = SYS->GPA_MFPL & SYS_GPA_MFPL_PA6MFP_Msk;
    g_u32MFP_LV1 = SYS->GPA_MFPL & SYS_GPA_MFPL_PA7MFP_Msk;

    SYS->GPF_MFPL =  (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF5MFP_Msk) | SYS_GPF_MFPL_PF5MFP_GPIO;
    SYS->GPA_MFPH = ((SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) | (SYS_GPA_MFPH_PA9MFP_GPIO | SYS_GPA_MFPH_PA10MFP_GPIO));

    SYS->GPA_MFPL = ((SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk)) | (SYS_GPA_MFPL_PA6MFP_GPIO | SYS_GPA_MFPL_PA7MFP_GPIO));

    ICP_PIN_RST = (u32ResetActiveLevel == ICP_RESET_ACTIVE_LOW)? 1 : 0;
    ICP_PIN_CLK = 0;
    ICP_PIN_DAT = 0;

    ICP_PIN_LV0 = 1;
    ICP_PIN_LV1 = 0;

    ICP_SetGPIOMode(PF,  5, GPIO_MODE_OUTPUT);
    ICP_SetGPIOMode(PA, 10, GPIO_MODE_OUTPUT);
    ICP_SetGPIOMode(PA,  9, GPIO_MODE_OUTPUT);

    ICP_SetGPIOMode(PA,  6, GPIO_MODE_OUTPUT);
    ICP_SetGPIOMode(PA,  7, GPIO_MODE_OUTPUT);
}

void ICP_UnInit()
{
    ICP_SetGPIOMode(PF,  5, g_u32IOMode_RST);
    ICP_SetGPIOMode(PA, 10, g_u32IOMode_CLK);
    ICP_SetGPIOMode(PA,  9, g_u32IOMode_DAT);

    ICP_SetGPIOMode(PA,  6, g_u32IOMode_LV0);
    ICP_SetGPIOMode(PA,  7, g_u32IOMode_LV1);

    ICP_PIN_RST = g_u32IOLevel_RST;
    ICP_PIN_CLK = g_u32IOLevel_CLK;
    ICP_PIN_DAT = g_u32IOLevel_DAT;

    ICP_PIN_LV0 = g_u32IOLevel_LV0;
    ICP_PIN_LV1 = g_u32IOLevel_LV1;

    SYS->GPF_MFPL =  (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF5MFP_Msk) | g_u32MFP_RST;
    SYS->GPA_MFPH = ((SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) | (g_u32MFP_CLK | g_u32MFP_DAT));

    SYS->GPA_MFPL = ((SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk)) | (g_u32MFP_LV0 | g_u32MFP_LV1));
}

/* Adjust Delay Time & Hold Time */
void ICP_SetMassEraseTime(uint32_t u32DelayTime, uint32_t u32HoldTime)
{
    g_ICPTiming.u32MassEraseDelayTime = u32DelayTime;
    g_ICPTiming.u32MassEraseHoldTime  = u32HoldTime;
}

void ICP_SetPageEraseTime(uint32_t u32DelayTime, uint32_t u32HoldTime)
{
    g_ICPTiming.u32PageEraseDelayTime = u32DelayTime;
    g_ICPTiming.u32PageEraseHoldTime  = u32HoldTime;
}

void ICP_SetProgramTime(uint32_t u32DelayTime, uint32_t u32HoldTime)
{
    g_ICPTiming.u32ProgramDelayTime = u32DelayTime;
    g_ICPTiming.u32ProgramHoldTime  = u32HoldTime;
}

/* Read ID */
uint32_t ICP_ReadCID()
{
    uint32_t u32CID;

    ICP_SendCommand(FMC_ISPCMD_READ_CID, 0x00);

    u32CID = ICP_ReadData(1);

    return u32CID;
}

uint32_t ICP_ReadDID()
{
    uint32_t u32DID;

    ICP_SendCommand(FMC_ISPCMD_READ_DID, 0x00);

    u32DID = ICP_ReadData(1);

    return u32DID;
}

uint32_t ICP_ReadPID()
{
    uint32_t u32PID;

    ICP_SendCommand(FMC_ISPCMD_READ_DID, 0x04);

    u32PID = ICP_ReadData(1);

    return u32PID;
}

uint32_t ICP_ReadUID(uint32_t u32Index)
{
    uint32_t u32UID;

    if (u32Index >= 3)
        return 0xFFFFFFFF;

    ICP_SendCommand(FMC_ISPCMD_READ_UID, (u32Index << 2));

    u32UID = ICP_ReadData(1);

    return u32UID;
}

uint32_t ICP_ReadUCID(uint32_t u32Index)
{
    uint32_t u32UCID;

    if (u32Index >= 4)
        return 0xFFFFFFFF;

    ICP_SendCommand(FMC_ISPCMD_READ_UID, (u32Index << 2) + 0x10);

    u32UCID = ICP_ReadData(1);

    return u32UCID;
}

void ICP_ModeEntry()
{
    uint32_t i, u32CID, u32DID;

    ICP_SetReset(TRUE);

    CLK_SysTickDelay(500);

    ICP_PIN_DAT = 1;

    for (i = 0; i < 50; i++)
    {
        ICP_CLOCK_CYCLE();
    }

    ICP_WriteBits(ICP_ENTRY_CODE, 24);

    CLK_SysTickDelay(5);

    // Read CID
    u32CID = ICP_ReadCID();

    // Read DID
    u32DID = ICP_ReadDID();
}

void ICP_ModeExit()
{
    CLK_SysTickDelay(110);

    ICP_SetReset(FALSE);
}

void ICP_PageEraseFlash(uint32_t u32Addr)
{
    ICP_SendCommand(FMC_ISPCMD_PAGE_ERASE, u32Addr);
    ICP_WriteData(0xFFFFFFFF, 1, g_ICPTiming.u32PageEraseDelayTime, g_ICPTiming.u32PageEraseHoldTime);
}

void ICP_SPROMErase(uint32_t u32Addr)
{
    ICP_SendCommand(FMC_ISPCMD_PAGE_ERASE, u32Addr);
    ICP_WriteData(0x0055AA03, 1, g_ICPTiming.u32PageEraseDelayTime, g_ICPTiming.u32PageEraseHoldTime);
}

void ICP_XOMErase(uint32_t u32Addr, uint32_t u32PageCount)
{
    ICP_SendCommand(FMC_ISPCMD_PAGE_ERASE, u32Addr);
    ICP_WriteData(0x0055AA03, 1, g_ICPTiming.u32PageEraseDelayTime * u32PageCount, g_ICPTiming.u32PageEraseHoldTime);
}

void ICP_MassErase()
{
    ICP_SendCommand(FMC_ISPCMD_MASS_ERASE, 0x00000000);
    ICP_WriteData(0x00000000, 1, g_ICPTiming.u32MassEraseDelayTime, g_ICPTiming.u32MassEraseHoldTime);
}

void ICP_BankErase(uint32_t u32Addr)
{
    ICP_SendCommand(FMC_ISPCMD_BANK_ERASE, u32Addr);
    ICP_WriteData(0xFFFFFFFF, 1, g_ICPTiming.u32MassEraseDelayTime, g_ICPTiming.u32MassEraseHoldTime);
}

void ICP_BlockErase(uint32_t u32Addr)
{
    ICP_SendCommand(FMC_ISPCMD_BLOCK_ERASE, u32Addr);
    ICP_WriteData(0xFFFFFFFF, 1, 4 * g_ICPTiming.u32PageEraseDelayTime, 4 * g_ICPTiming.u32PageEraseHoldTime);
}

uint32_t ICP_ProgramFlash(uint32_t u32StartAddr, uint32_t u32ByteSize, uint32_t *u32Buffer)
{
    uint32_t i, u32DelayTime, u32HoldTime;

    if ((u32StartAddr % 4) || (u32ByteSize == 0) || (u32ByteSize % 4) || ((uint32_t)u32Buffer % 4))
        return 0xFFFFFFFF;

    u32DelayTime = g_ICPTiming.u32ProgramDelayTime;
    u32HoldTime  = g_ICPTiming.u32ProgramHoldTime;

    ICP_SendCommand(FMC_ISPCMD_PROGRAM, u32StartAddr);

    if (u32ByteSize > 4)
    {
        for (i = 0; i < ((u32ByteSize/4) - 1); i++)
        {
            ICP_WriteData(u32Buffer[i], 0, u32DelayTime, u32HoldTime);
        }

        ICP_WriteData(u32Buffer[i], 1, u32DelayTime, u32HoldTime);
    }
    else
    {
        ICP_WriteData(u32Buffer[0], 1, u32DelayTime, u32HoldTime);
    }

    return u32ByteSize;
}

// *    Return Value:   (adr+sz) - OK, Failed Address
uint32_t ICP_ProgramVerifyFlash(uint32_t u32StartAddr, uint32_t u32ByteSize, uint32_t *u32Buffer)
{
    uint32_t i, u32Data, u32DelayTime, u32HoldTime;

    if ((u32StartAddr % 4) || (u32ByteSize == 0) || (u32ByteSize % 4) || ((uint32_t)u32Buffer % 4))
        return 0xFFFFFFFF;

    u32DelayTime = g_ICPTiming.u32ProgramDelayTime;
    u32HoldTime  = g_ICPTiming.u32ProgramHoldTime;

    if (u32ByteSize > 4)
    {
        // Program
        ICP_SendCommand(FMC_ISPCMD_PROGRAM, u32StartAddr);

        for (i = 0; i < ((u32ByteSize/4) - 1); i++)
        {
            ICP_WriteData(u32Buffer[i], 0, u32DelayTime, u32HoldTime);
        }

        ICP_WriteData(u32Buffer[i], 1, u32DelayTime, u32HoldTime);

        // Verify
        ICP_SendCommand(FMC_ISPCMD_READ, u32StartAddr);

        for (i = 0; i < ((u32ByteSize/4) - 1); i++)
        {
            u32Data = ICP_ReadData(0);

            if (u32Data != u32Buffer[i])
                return (u32StartAddr + (i * 4));
        }

        u32Data = ICP_ReadData(1);

        if (u32Data != u32Buffer[i])
            return (u32StartAddr + (i * 4));
    }
    else
    {
        ICP_SendCommand(FMC_ISPCMD_PROGRAM, u32StartAddr);

        ICP_WriteData(u32Buffer[0], 1, u32DelayTime, u32HoldTime);

        ICP_SendCommand(FMC_ISPCMD_READ, u32StartAddr);

        u32Data = ICP_ReadData(1);

        if (u32Data != u32Buffer[0])
            return u32StartAddr;
    }

    return (u32StartAddr + u32ByteSize);
}

void ICP_ProgramConfigWithCRC(uint32_t u32Config0, uint32_t u32Config1)
{
    uint32_t i, u32DelayTime, u32HoldTime;
    uint32_t u32Config[4];

    u32Config[0] = u32Config0;
    u32Config[1] = u32Config1;
    u32Config[2] = 0xFFFFFFFF;
    u32Config[3] = GetCRCValue(u32Config, 3);

    u32DelayTime = g_ICPTiming.u32ProgramDelayTime;
    u32HoldTime  = g_ICPTiming.u32ProgramHoldTime;

    for (i = 0; i < 4; i++)
    {
        ICP_SendCommand(FMC_ISPCMD_PROGRAM, FMC_CONFIG_BASE + (i * 4));
        ICP_WriteData(u32Config[i], 1, u32DelayTime, u32HoldTime);
    }
}

uint32_t ICP_ReadFlash(uint32_t u32StartAddr, uint32_t u32ByteSize, uint32_t *u32Buffer)
{
    uint32_t i;

    if ((u32StartAddr % 4) || (u32ByteSize == 0) || (u32ByteSize % 4) || ((uint32_t)u32Buffer % 4))
        return 0xFFFFFFFF;

    ICP_SendCommand(FMC_ISPCMD_READ, u32StartAddr);

    if (u32ByteSize > 4)
    {
        for (i = 0; i < ((u32ByteSize/4) - 1); i++)
        {
            u32Buffer[i] = ICP_ReadData(0);
        }

        u32Buffer[i] = ICP_ReadData(1);
    }
    else
    {
        u32Buffer[0] = ICP_ReadData(1);
    }

    return u32ByteSize;
}
