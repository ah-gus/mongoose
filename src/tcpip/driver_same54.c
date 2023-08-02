#include "tcpip.h"

#if MG_ENABLE_TCPIP && defined(MG_ENABLE_DRIVER_SAME54) && \
    MG_ENABLE_DRIVER_SAME54

#include <sam.h>

#undef BIT
#define BIT(x) ((uint32_t) 1 << (x))
#define ETH_PKT_SIZE 1536  // Max frame size
#define GMAC_DESC_CNT 4     // Descriptors count
#define GMAC_DS 2           // Descriptor size (words)

static uint8_t s_rxbuf[GMAC_DESC_CNT][ETH_PKT_SIZE];  // RX ethernet buffers
static uint8_t s_txbuf[GMAC_DESC_CNT][ETH_PKT_SIZE];  // TX ethernet buffers
static uint32_t s_rxdesc[GMAC_DESC_CNT][GMAC_DS];      // RX descriptors
static uint32_t s_txdesc[GMAC_DESC_CNT][GMAC_DS];      // TX descriptors
static uint8_t s_txno;                               // Current TX descriptor
static uint8_t s_rxno;                               // Current RX descriptor

static struct mg_tcpip_if *s_ifp;  // MIP interface
enum { PHY_ADDR = 0, PHY_BCR = 0, PHY_BSR = 1};

static uint16_t eth_read_phy(uint8_t addr, uint8_t reg) {
  GMAC_REGS->GMAC_MAN = GMAC_MAN_CLTTO_Msk | GMAC_MAN_OP(2) |   // Setting the read operation
                        GMAC_MAN_WTN(2) | GMAC_MAN_PHYA(addr) | // PHY address
                        GMAC_MAN_REGA(reg);   // Setting the register
  while (!(GMAC_REGS->GMAC_NSR & GMAC_NSR_IDLE_Msk)); // Waiting until the read op is complete
  return GMAC_REGS->GMAC_MAN & GMAC_MAN_DATA_Msk;   // Getting the read value
}

#if 1
static void eth_write_phy(uint8_t addr, uint8_t reg, uint16_t val) {
  GMAC_REGS->GMAC_MAN = GMAC_MAN_CLTTO_Msk | GMAC_MAN_OP(1) |   // Setting the write operation
                        GMAC_MAN_WTN(2) | GMAC_MAN_PHYA(addr) | // PHY address
                        GMAC_MAN_REGA(reg) | GMAC_MAN_DATA(val);  // Setting the register
  while (!(GMAC_REGS->GMAC_NSR & GMAC_NSR_IDLE_Msk)); // Waiting until the write op is complete
}
#endif

int get_clock_rate(struct mg_tcpip_driver_same54_data *d) {
  if (d && d->mdc_cr >= 0 && d->mdc_cr <= 5) {
    return d->mdc_cr;
  } else {
    // get MCLK from GCLK_GENERATOR 0
    uint32_t div = 512;
    uint32_t mclk;
    if (!(GCLK_REGS->GCLK_GENCTRL[0] & GCLK_GENCTRL_DIVSEL_Msk)) {
      div = ((GCLK_REGS->GCLK_GENCTRL[0] & 0x00FF0000) >> 16);
      if (div == 0) div = 1;
    }
    switch (GCLK_REGS->GCLK_GENCTRL[0] & GCLK_GENCTRL_SRC_Msk) {
      case GCLK_GENCTRL_SRC_XOSC0_Val:
        mclk = 32000000UL; /* 32MHz */
        break;
      case GCLK_GENCTRL_SRC_XOSC1_Val:
        mclk = 32000000UL; /* 32MHz */
        break;
      case GCLK_GENCTRL_SRC_OSCULP32K_Val:
        mclk = 32000UL;
        break;
      case GCLK_GENCTRL_SRC_XOSC32K_Val:
        mclk = 32000UL;
        break;
      case GCLK_GENCTRL_SRC_DFLL_Val:
        mclk = 48000000UL; /* 48MHz */
        break;
      case GCLK_GENCTRL_SRC_DPLL0_Val:
        mclk = 200000000UL; /* 200MHz */
        break;
      case GCLK_GENCTRL_SRC_DPLL1_Val:
        mclk = 200000000UL; /* 200MHz */
        break;
      default:
        mclk = 200000000UL; /* 200MHz */
    }

    mclk /= div;
    uint8_t crs[] = {0, 1, 2, 3, 4, 5};          // GMAC->NCFGR::CLK values
    uint8_t dividers[] = {8, 16, 32, 48, 64, 128};  // Respective CLK dividers
    for (int i = 0; i < 6; i++) {
      if (mclk / dividers[i] <= 2375000UL /* 2.5MHz - 5% */) {
        return crs[i];
      }
    }

    return 5;
  }
}

static bool mg_tcpip_driver_same54_init(struct mg_tcpip_if *ifp) {
  struct mg_tcpip_driver_same54_data *d =
      (struct mg_tcpip_driver_same54_data *) ifp->driver_data;
  s_ifp = ifp;

  MCLK_REGS->MCLK_APBCMASK |= MCLK_APBCMASK_GMAC_Msk; // Enabling GMAC bus clocks
  MCLK_REGS->MCLK_AHBMASK |= MCLK_AHBMASK_GMAC_Msk;
  GMAC_REGS->GMAC_NCFGR = GMAC_NCFGR_CLK(get_clock_rate(d));  // Set MDC divider
  GMAC_REGS->GMAC_NCR = 0;                            // Disable RX & TX
  GMAC_REGS->GMAC_NCR |= GMAC_NCR_MPE_Msk;            // Enable MDC & MDIO

  // Init RX descriptors
  GMAC_REGS->GMAC_DCFGR = GMAC_DCFGR_DRBS(0x18);
  for (int i = 0; i < GMAC_DESC_CNT; i++) {
    s_rxdesc[i][0] = (uint32_t) s_rxbuf[i];  // Point to data buffer (bits [31:2])
    s_rxdesc[i][1] = 0;
  }
  s_rxdesc[GMAC_DESC_CNT - 1][0] |= BIT(1); // Marking last rx descriptor

  // Init TX descriptors
  for (int i = 0; i < GMAC_DESC_CNT; i++) {
    s_txdesc[i][0] = (uint32_t) s_txbuf[i];  // Point to data buffer
    s_txdesc[i][1] = BIT(31); // Setting the OWN bit
  }
  s_txdesc[GMAC_DESC_CNT - 1][1] |= BIT(30); // Marking last tx descriptor

  // let the controller know about the descriptor addresses
  GMAC_REGS->GMAC_RBQB = (uint32_t) s_rxdesc;
  GMAC_REGS->GMAC_TBQB = (uint32_t) s_txdesc;

  // configure MAC
  GMAC_REGS->SA[0].GMAC_SAT = ((uint32_t) ifp->mac[5] << 8U) | ifp->mac[4];
  GMAC_REGS->SA[0].GMAC_SAB = (uint32_t) (ifp->mac[3] << 24) |
                 ((uint32_t) ifp->mac[2] << 16) |
                 ((uint32_t) ifp->mac[1] << 8) | ifp->mac[0];

  // Select RMII operation mode
  GMAC_REGS->GMAC_UR &= ~GMAC_UR_MII_Msk;

  //Configure the receive filter
  GMAC_REGS->GMAC_NCFGR |= GMAC_NCFGR_MAXFS_Msk | GMAC_NCFGR_MTIHEN_Msk |
                           GMAC_NCFGR_SPD_Msk | GMAC_NCFGR_FD_Msk ;

  // Clear transmit status register
  GMAC_REGS->GMAC_TSR = GMAC_TSR_HRESP_Msk | GMAC_TSR_UND_Msk |
                        GMAC_TSR_TXCOMP_Msk | GMAC_TSR_TFC_Msk |
                        GMAC_TSR_TXGO_Msk | GMAC_TSR_RLE_Msk |
                        GMAC_TSR_COL_Msk | GMAC_TSR_UBR_Msk;

  // Clear receive status register
  GMAC_REGS->GMAC_RSR = GMAC_RSR_HNO_Msk | GMAC_RSR_RXOVR_Msk |
                        GMAC_RSR_REC_Msk | GMAC_RSR_BNA_Msk;

  // First disable all GMAC interrupts
  GMAC_REGS->GMAC_IDR = ~0U;

  // Only the desired ones are enabled
  GMAC_REGS->GMAC_IER = GMAC_IER_HRESP_Msk | GMAC_IER_ROVR_Msk |
                        GMAC_IER_RXUBR_Msk | GMAC_IER_RCOMP_Msk;

  NVIC_EnableIRQ(GMAC_IRQn);

  // Enable the GMAC to transmit and receive data
  GMAC_REGS->GMAC_NCR |= GMAC_NCR_TXEN_Msk | GMAC_NCR_RXEN_Msk;

  return true;
}

static size_t mg_tcpip_driver_same54_tx(const void *buf, size_t len,
                                       struct mg_tcpip_if *ifp) {
  MG_INFO(("GMAC_TSR: 0x%x", GMAC_REGS->GMAC_TSR));
  if (len > sizeof(s_txbuf[s_txno])) {
    MG_ERROR(("Frame too big, %ld", (long) len));
    len = 0;  // Frame is too big
  } else if (!(s_txdesc[s_txno][1] & BIT(31))) {
    ifp->nerr++;
    MG_ERROR(("No free descriptors"));
    len = 0;  // All descriptors are busy, fail
  } else {
    //mg_hexdump(buf, len);
    memcpy(s_txbuf[s_txno], buf, len);     // Copy data
    if (++s_txno >= GMAC_DESC_CNT) {
      s_txdesc[GMAC_DESC_CNT - 1][1] = (len & (BIT(14) - 1)) | BIT(15)| BIT(30);
      s_txno = 0;
      MG_INFO(("s_tx_no: %d, tx_desc.status: 0x%x", GMAC_DESC_CNT - 1, s_txdesc[GMAC_DESC_CNT - 1][1]));
    } else {
      s_txdesc[s_txno - 1][1] = (len & (BIT(14) - 1)) | BIT(15);
      MG_INFO(("s_tx_no: %d, tx_desc.status: 0x%x", s_txno - 1, s_txdesc[s_txno - 1][1]));
    }
  }
  __DSB();  // Ensure descriptors have been written
  GMAC_REGS->GMAC_NCR |= GMAC_NCR_TSTART_Msk; // Enable transmission
  return len;
}

static bool mg_tcpip_driver_same54_up(struct mg_tcpip_if *ifp) {
  uint16_t bsr = eth_read_phy(PHY_ADDR, PHY_BSR);
  bool up = bsr & BIT(2) ? 1 : 0;
  (void) ifp;
  return up;
}

void GMAC_Handler(void) {
  uint32_t isr = GMAC_REGS->GMAC_ISR;
  uint32_t rsr = GMAC_REGS->GMAC_RSR;
  uint32_t tsr = GMAC_REGS->GMAC_TSR;
  MG_INFO(("ISR: 0x%x, TSR: 0x%x, RSR: 0x%x", isr, tsr, rsr));
  if (isr & GMAC_ISR_RCOMP_Msk) {
    if (rsr & GMAC_ISR_RCOMP_Msk) {
      for (int i = 0; i < 10; i++) {
        if ((s_rxdesc[s_rxno][0] & BIT(0)) == 0) break;
        uint32_t len = s_rxdesc[s_rxno][1] & (BIT(13) - 1);
        size_t offset = (GMAC_REGS->GMAC_NCFGR & GMAC_NCFGR_RXBUFO_Msk) >> GMAC_NCFGR_RXBUFO_Pos;
        //mg_hexdump(s_rxbuf[s_rxno] + offset, len);
        mg_tcpip_qwrite(s_rxbuf[s_rxno] + offset, len, s_ifp);
        s_rxdesc[s_rxno][0] &= ~BIT(0);  // Disown
        if (++s_rxno >= GMAC_DESC_CNT) s_rxno = 0;
      }
    }
  }

  if ((tsr & (GMAC_TSR_HRESP_Msk | GMAC_TSR_UND_Msk | GMAC_TSR_TXCOMP_Msk |
              GMAC_TSR_TFC_Msk | GMAC_TSR_TXGO_Msk | GMAC_TSR_RLE_Msk |
              GMAC_TSR_COL_Msk | GMAC_TSR_UBR_Msk)) != 0) {
    MG_INFO((" --> %#x %#x", s_txdesc[s_txno][1], tsr));
    if (!(s_txdesc[s_txno][1] & BIT(31))) s_txdesc[s_txno][1] |= BIT(31);
  }

  GMAC_REGS->GMAC_RSR = rsr;
  GMAC_REGS->GMAC_TSR = tsr;
}

struct mg_tcpip_driver mg_tcpip_driver_same54 = {mg_tcpip_driver_same54_init,
                                                mg_tcpip_driver_same54_tx, NULL,
                                                mg_tcpip_driver_same54_up};
#endif
