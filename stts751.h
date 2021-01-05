/*
 MIT License

Copyright (c) 2020,2021 Konomu Abe

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef STTS751_H
#define	STTS751_H

#ifdef	__cplusplus
extern "C" {
#endif

#define STTS751_I2C_ADDRESS (0x72/2)
    
#define STTS751_REGISTER_ADDRESS_STATUS    (0x01)
#define STTS751_REGISTER_ADDRESS_HI    (0x00)
#define STTS751_REGISTER_ADDRESS_LO    (0x02)

    
#ifdef	__cplusplus
}
#endif

#endif	/* STTS751_H */

