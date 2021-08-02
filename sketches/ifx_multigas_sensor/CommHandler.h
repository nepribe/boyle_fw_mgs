#ifndef __COMMHANDLER_H__
#define __COMMHANDLER_H__

bool CommRx();

bool CommTx(const char * msg);
bool CommTxError(char *  error_msg);
bool CommTxData(uint8_t *d, uint8_t n);
bool CommTxDataBoyleAutoScale(unsigned int *d);
bool CommTxDataBoyleRef(uint8_t *d);
bool CommTxDataBoyle(uint8_t *d);
bool CommTxDataDps368(double temp, double pres);
bool CommTxDatadpsSht31(double temp, double humidity);
bool CommTxDatadpsAplhasense(double temp, double NO2, double O3);

#endif
