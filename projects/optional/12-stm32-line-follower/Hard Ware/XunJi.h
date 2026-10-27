#ifndef __XUNJI_H
#define __XUNJI_H

#define OFF 0
#define Zheng_Chang 1
#define Pian_Zuo 2
#define Pian_You 3
static unsigned char Zhong = 2;
static unsigned char Zuo = 2;
static unsigned char You = 2;

void Xun_Ji_Init(void);

unsigned char Fang_Xiang(void);

unsigned char ZhuangTai_Zhong(void);

unsigned char ZhuangTai_Zuo(void);

unsigned char ZhuangTai_You(void);

#endif

