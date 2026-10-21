#include "xl430.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t regs[3][160], response[256], last_tx[128];
static size_t response_size, response_at, tx_size;
static bool bad_crc, wrong_mode;
static unsigned transmissions;
uint32_t HAL_GetTick(void) {return 0;}
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *h) {(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *h) {(void)h;return HAL_OK;}
static uint16_t crc(const uint8_t *p,size_t n) {
    uint32_t value=0;
    for(size_t i=0;i<n;i++) {
        value^=(uint32_t)p[i]<<8;
        for(unsigned j=0;j<8;j++) value=(value<<1)^((value&0x8000)?0x8005:0);
        value&=0xffff;
    }
    return (uint16_t)value;
}
static void reply(uint8_t id,const uint8_t *data,size_t n) {
    uint8_t p[128]={255,255,253,0,id,0,0,0x55,0};size_t end=9;
    for(size_t i=0;i<n;i++) {
        p[end++]=data[i];
        if(i>=2 && data[i-2]==255 && data[i-1]==255 && data[i]==253) p[end++]=253;
    }
    p[5]=(uint8_t)(end+2-7);
    uint16_t sum=crc(p,end);p[end++]=(uint8_t)sum;p[end++]=(uint8_t)(sum>>8);
    if(bad_crc)p[end-1]^=1;
    memcpy(response+response_size,p,end);response_size+=end;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t ms) {
    (void)h;(void)ms;++transmissions;
    memcpy(last_tx,p,n);tx_size=n;
    assert(n==(unsigned)p[5]+((unsigned)p[6]<<8)+7);
    assert(crc(p,n-2)==((uint16_t)p[n-2]|((uint16_t)p[n-1]<<8)));
    uint8_t data[64];size_t len=0;
    for(size_t i=7;i<n-2U;i++) {
        data[len++]=p[i];
        if(i>=9 && p[i-2]==255 && p[i-1]==255 && p[i]==253) {assert(p[++i]==253);}
    }
    response_size=response_at=0;
    uint8_t id=p[4];
    if(data[0]==1) {const uint8_t model[]={0x24,4,42};reply(id,model,3);return HAL_OK;}
    unsigned addr=(unsigned)data[1]|((unsigned)data[2]<<8);
    assert(addr<160);
    if(data[0]==3) {
        assert(id<3 && addr+len-3<=160);
        if(addr<64)assert(regs[id][64]==0);
        memcpy(&regs[id][addr],data+3,len-3);
        if(addr==11) {memset(&regs[id][108],0,8);if(wrong_mode)regs[id][11]=3;}
        reply(id,NULL,0);
    } else if(data[0]==2) {
        unsigned count=(unsigned)data[3]|((unsigned)data[4]<<8);
        assert(addr+count<=160);reply(id,&regs[id][addr],count);
    } else if(data[0]==0x83) {
        assert(id==254 && len==15 && data[3]==4 && data[4]==0);
        memcpy(&regs[data[5]][addr],data+6,4);memcpy(&regs[data[10]][addr],data+11,4);
    } else if(data[0]==0x82) {
        assert(id==254 && len==7 && data[3]==8);
        reply(data[5],&regs[data[5]][addr],8);reply(data[6],&regs[data[6]][addr],8);
    } else assert(0);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t ms) {
    (void)h;(void)ms;assert(n==1);
    if(response_at==response_size)return HAL_TIMEOUT;
    *p=response[response_at++];return HAL_OK;
}
static int32_t signed_reg(uint8_t id,unsigned addr) {
    uint32_t raw=(uint32_t)regs[id][addr]|((uint32_t)regs[id][addr+1]<<8)|
        ((uint32_t)regs[id][addr+2]<<16)|((uint32_t)regs[id][addr+3]<<24);
    int32_t value;memcpy(&value,&raw,4);return value;
}
int main(void) {
    UART_HandleTypeDef uart={0};XL430_Handle_t l,r;
    assert(XL430_Init(&l,&uart,1)==XL430_OK && XL430_Init(&r,&uart,2)==XL430_OK);
    assert(XL430_Ping(&l,NULL,NULL)==XL430_OK);
    const uint8_t official_ping[]={255,255,253,0,1,3,0,1,0x19,0x4e};
    assert(tx_size==sizeof(official_ping) && memcmp(last_tx,official_ping,tx_size)==0);
    regs[1][10]=0x0d;regs[1][64]=1;
    assert(XL430_ConfigurePositionControl(&l,20,10,50)==XL430_OK);
    assert(regs[1][64]==0 && regs[1][10]==1 && regs[1][11]==4 && regs[1][98]==50);
    assert(signed_reg(1,108)==10 && signed_reg(1,112)==20);
    assert(XL430_SetGoalPosition(&l,-131073)==XL430_OK);
    assert(signed_reg(1,116)==-131073 && tx_size==17); /* FF FF FD FF needs stuffing. */
    assert(XL430_SyncWriteGoalPosition(&l,-131073,&r,1048575)==XL430_OK);
    assert(signed_reg(1,116)==-131073 && signed_reg(2,116)==1048575 && tx_size==25);
    memcpy(&regs[1][132],&regs[1][116],4);memcpy(&regs[2][132],&regs[2][116],4);
    XL430_WheelState_t ls,rs;
    assert(XL430_SyncReadWheelState(&l,&ls,&r,&rs)==XL430_OK);
    assert(ls.position==-131073 && rs.position==1048575);
    bad_crc=true;assert(XL430_SyncReadWheelState(&l,&ls,&r,&rs)==XL430_ERROR_CRC);bad_crc=false;
    assert(XL430_ConfigureVelocityControl(&l,50)==XL430_OK);
    assert(regs[1][11]==1 && regs[1][64]==1 && signed_reg(1,104)==0);
    wrong_mode=true;assert(XL430_ConfigureVelocityControl(&l,50)==XL430_ERROR_DEVICE);
    assert(regs[1][64]==0);wrong_mode=false;
    unsigned before=transmissions;
    assert(XL430_SetGoalPosition(&l,1048576)==XL430_ERROR_INVALID_ARGUMENT);
    assert(XL430_SetGoalPosition(&l,-1048576)==XL430_ERROR_INVALID_ARGUMENT);
    assert(XL430_ConfigurePositionControl(&l,0,10,50)==XL430_ERROR_INVALID_ARGUMENT);
    assert(XL430_ConfigurePositionControl(&l,20,0,50)==XL430_ERROR_INVALID_ARGUMENT);
    assert(XL430_SyncWriteGoalPosition(&l,0,&l,0)==XL430_ERROR_INVALID_ARGUMENT);
    assert(transmissions==before);
    puts("PASS XL430 wire: official Ping CRC, position/profile registers, mode readback, signed goals, Sync Write, TX/RX byte stuffing, CRC and bounds");
}
