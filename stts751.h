/* 
 * File:   stts751.h
 * Author: konomu
 *
 * Created on 2019/04/21, 7:29
 */

#ifndef STTS751_H
#define	STTS751_H

#ifdef	__cplusplus
extern "C" {
#endif

#define STTS751_REGISTER_ADDRESS_STATUS    (0x01)
#define STTS751_REGISTER_ADDRESS_HI    (0x00)
#define STTS751_REGISTER_ADDRESS_LO    (0x02)

void STTS751_test_sync(void);
unsigned short STTS751_read_regsiter_sync(unsigned char regsiter);
void STTS751_read_regsiter(unsigned char regsiter,unsigned short *d);
void STTS751_test(void);

    
#ifdef	__cplusplus
}
#endif

#endif	/* STTS751_H */

