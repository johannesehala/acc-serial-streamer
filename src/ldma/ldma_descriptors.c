/**
 * ldma_descriptors.c
 * 
 * @brief Here descriptors are created for memory to UART transfer. 
 * 
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2022
 */

#include "ldma_config.h"
#include "ldma_descriptors.h"

LDMA_Descriptor_t msgToUartDsc[NUM_LDMA_DESCRIPTORS];
LDMA_Descriptor_t tokenToUartDsc;

/**
 * This is a linked descriptor. An interrupt is generated when the last descriptor
 * finishes.
 *
 * This descriptor is set up to transfer from memory to UART 2 bytes at a time. 
 * Notice, that destination address is USARTn->TXDOUBLE, which is a two byte 
 * FIFO in the UART transfer area. Destination address is not incremented.
 * 
 * Transfer unit size (.xfer.size) is half-word (because ADC data is 16 bits).
 *
 * Transfer block size (.xfer.blockSize) is 1, which means one unit of data is
 * transferred during a block. LDMA does not arbitrate during the transfer of 
 * a block of data, i.e. other LDMA channels can't use LDMA while block of data
 * is being serviced. Only after block transfer has completed will LDMA check 
 * if other LDMA channels need servicing.
 *
 * Transfer count (.xfer.xferCnt) is the number of units to be transferred 
 * within a descriptor.
 *
 * Byte swap is used, because memory is in little-endian order in the gecko memory. 
 *
 * The first descriptor uses absolute adderssing mode for source address. This 
 * is a requirement. The second descriptor uses relative adderssing mode for 
 * source address. This means that the second descriptors source address is
 * the address of the first descriptors last transfer (eg. if first descriptor
 * source address was 0x00001111 and the descriptor transferred 40 half words
 * then the address of the last transfer would be something like 0x00001125, 
 * since memory area is 32bit and 2 half words occupy one memory slot so the 
 * initial address gets incremented 20 memory slots). Since source address
 * increment is used, the last address actually becomes the address of the
 * next data value to be transferred (0x00001127) and 
 * no offset is needed (.xfer.srcAddr = 0).
 */
LDMA_Descriptor_t* msg_descriptor_config(uint32_t* bufAddr, uint32_t payload_len_bytes)
{
    uint8_t i;
    uint32_t transfer_count = (uint32_t)(payload_len_bytes/2); // Using half-word (16-bit) transfers
    
    // Create descriptors.
    for (i = 0; i < NUM_LDMA_DESCRIPTORS; i++)
    {
        msgToUartDsc[i].xfer.structType     = ldmaCtrlStructTypeXfer;
        msgToUartDsc[i].xfer.structReq      = 0; // Transfer started by USART signal, not descr. load.
        msgToUartDsc[i].xfer.byteSwap       = 0; // Memory is in little-endian order, but network bytes are already swapped, so don't swap bytes.
        msgToUartDsc[i].xfer.blockSize      = ldmaCtrlBlockSizeUnit1; // Smallest block so as not to starve other DMA channels.
        msgToUartDsc[i].xfer.reqMode        = ldmaCtrlReqModeBlock; // Recommended for peripheral transfer.
        msgToUartDsc[i].xfer.decLoopCnt     = 0; // Descriptor is not looped.
        msgToUartDsc[i].xfer.ignoreSrec     = 1; // Page 519 efr32xg1 reference manual r1.1
        msgToUartDsc[i].xfer.srcInc         = ldmaCtrlSrcIncOne;
        msgToUartDsc[i].xfer.size           = ldmaCtrlSizeHalf; // Transfer 2 bytes at a time (half-word).
        msgToUartDsc[i].xfer.dstInc         = ldmaCtrlDstIncNone; // Don't increment UART TX buffer.
        msgToUartDsc[i].xfer.dstAddrMode    = ldmaCtrlDstAddrModeAbs;
        msgToUartDsc[i].xfer.dstAddr        = (uint32_t)&USART_FOR_LDMA->TXDOUBLE; // UART TX buffer address for half-word data.
        msgToUartDsc[i].xfer.linkAddr       = 4; // Point to next descriptor.
        msgToUartDsc[i].xfer.linkMode       = ldmaLinkModeRel;

        if (0 == i) // First descriptor.
        {
            msgToUartDsc[i].xfer.srcAddrMode    = ldmaCtrlSrcAddrModeAbs;
            msgToUartDsc[i].xfer.srcAddr        = (uint32_t)bufAddr; // Memory address
        }
        else // All but first.
        {
            msgToUartDsc[i].xfer.srcAddrMode    = ldmaCtrlSrcAddrModeRel;
            msgToUartDsc[i].xfer.srcAddr        = 0; // Offset from source address of previous transfer.
        }
        
        if ((NUM_LDMA_DESCRIPTORS - 1) == i) // Last descriptor.
        {
            msgToUartDsc[i].xfer.xferCnt    = (transfer_count % 2048) - 1; // One less then needed. See manual p214. Using half-word tranfer hence divide data length with 2.
            msgToUartDsc[i].xfer.doneIfs    = 1; // Generate interrupt after descriptor is done.
            msgToUartDsc[i].xfer.link       = 0; // Don't link to next.
        }
        else // All but last.
        {
            msgToUartDsc[i].xfer.xferCnt    = 2047; // Transfers 2048 units. See manual p214.
            msgToUartDsc[i].xfer.doneIfs    = 0; // No interrupt after descriptor is done.
            msgToUartDsc[i].xfer.link       = 1; // Link to next descriptor.
        }
    }
    return &msgToUartDsc[0];
}

LDMA_Descriptor_t* token_descriptor_config(uint32_t* bufAddr, uint32_t data_len_bytes)
{
    uint32_t transfer_count = (uint32_t)(data_len_bytes/2); // Using half-word (16-bit) transfers
    
    tokenToUartDsc.xfer.structType     = ldmaCtrlStructTypeXfer;
    tokenToUartDsc.xfer.structReq      = 0; // Transfer started by USART signal, not descr. load.
    tokenToUartDsc.xfer.byteSwap       = 1; // Memory is in little-endian order, so swap bytes.
    tokenToUartDsc.xfer.blockSize      = ldmaCtrlBlockSizeUnit1; // Smallest block so as not to starve other DMA channels.
    tokenToUartDsc.xfer.reqMode        = ldmaCtrlReqModeBlock; // Recommended for peripheral transfer.
    tokenToUartDsc.xfer.decLoopCnt     = 0; // Descriptor is not looped.
    tokenToUartDsc.xfer.ignoreSrec     = 1; // Page 519 efr32xg1 reference manual r1.1
    tokenToUartDsc.xfer.srcInc         = ldmaCtrlSrcIncOne;
    tokenToUartDsc.xfer.size           = ldmaCtrlSizeHalf; // Transfer 2 bytes at a time (half-word).
    tokenToUartDsc.xfer.dstInc         = ldmaCtrlDstIncNone; // Don't increment UART TX buffer.
    tokenToUartDsc.xfer.dstAddrMode    = ldmaCtrlDstAddrModeAbs;
    tokenToUartDsc.xfer.dstAddr        = (uint32_t)&USART_FOR_LDMA->TXDOUBLE; // UART TX buffer address for half-word data.
    tokenToUartDsc.xfer.linkAddr       = 4; // Point to next descriptor.
    tokenToUartDsc.xfer.linkMode       = ldmaLinkModeRel;
    tokenToUartDsc.xfer.srcAddrMode    = ldmaCtrlSrcAddrModeAbs;
    tokenToUartDsc.xfer.srcAddr        = (uint32_t)bufAddr; // Memory address
    tokenToUartDsc.xfer.xferCnt        = (transfer_count % 2048) - 1; // One less then needed. See manual p214. Using half-word tranfer hence divide data length with 2.
    tokenToUartDsc.xfer.doneIfs        = 1; // Generate interrupt after descriptor is done.
    tokenToUartDsc.xfer.link           = 0; // Don't link to next.
    
    return &tokenToUartDsc;
}
