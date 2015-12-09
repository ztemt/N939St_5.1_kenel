#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

#include <linux/string.h>
//#include <linux/slab.h>
#include <linux/gfp.h>
#include  <uapi/linux/types.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

//#include <math.h>

#define T4KB3_ERR -1

//static  uint16_t T4KB3_GOLDEN_R = 65;
//static  uint16_t T4KB3_GOLDEN_G = 90;
//static  uint16_t T4KB3_GOLDEN_B = 71;

static  uint16_t t4kb3_golden_r = 65;
static  uint16_t t4kb3_golden_g = 90;
static  uint16_t t4kb3_golden_b = 71;
  
  uint16_t t4kb3_af_macro_pos=500;
  uint16_t t4kb3_af_inifity_pos=100;

  uint16_t  t4kb3_af_macro_value = 0; 
  uint16_t  t4kb3_af_inifity_value = 0;
  uint16_t  t4kb3_af_otp_status = 0;


//uint8_t t4kb3_lsc_flag = 0;

typedef struct{
uint16_t lsc_err;
uint16_t awb_err;
uint16_t af_err;
uint16_t module_err;
}otp_err_detect;

typedef struct _otp_info{
uint8_t active_place[5];
uint8_t AF_macro[2];
uint8_t AF_infinity[2];
uint8_t module_info[15];
uint8_t awb[6];
otp_err_detect er;
}t4kb3_opt_info;

#define T4KB3_OTP_DATA_BEGIN_ADDR 0x3504
#define T4KB3_OTP_DATA_END_ADDR 0x3543

//static uint16_t t4kb3_otp_data[T4KB3_OTP_DATA_END_ADDR - T4KB3_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
//static uint16_t t4kb3_otp_data_backup[T4KB3_OTP_DATA_END_ADDR - T4KB3_OTP_DATA_BEGIN_ADDR + 1] = {0x00};

static int32_t t4kb3_sensor_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
	uint32_t addr, uint16_t *data,
	enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            addr,
            data, data_type);

    return rc;
}

static int32_t t4kb3_sensor_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
	uint32_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;

	pr_err("%s %d addr = %x, data = %x \n",__func__,__LINE__,addr,data);
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, addr, data, data_type);

    if (rc < 0) {
        msleep(100);
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, addr, data, data_type);
    }

	return rc;
}

void SET_T4KB3_REG(struct msm_sensor_ctrl_t *s_ctrl, uint32_t reg , uint16_t val)
{
	pr_err("%s %d reg = %x, val = %x \n",__func__,__LINE__,reg,val);
	t4kb3_sensor_i2c_write(s_ctrl,reg,val,MSM_CAMERA_I2C_BYTE_DATA);

}

void GET_T4KB3_REG(struct msm_sensor_ctrl_t *s_ctrl, uint32_t reg , uint16_t val)
{
	t4kb3_sensor_i2c_read(s_ctrl,reg,&val,MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("%s %d reg = %x, val = %x \n",__func__,__LINE__,reg,val);
}

static void t4kb3_otp_set_page(struct msm_sensor_ctrl_t *s_ctrl,uint16_t page)
{
	SET_T4KB3_REG(s_ctrl,0x3502,page);
}

static void t4kb3_otp_access(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t value = 1;
//	GET_T4KB3_REG(s_ctrl,0x3500,value);
	SET_T4KB3_REG(s_ctrl,0x3500,value|0x80);
}

static void t4kb3_otp_read_enable(struct msm_sensor_ctrl_t *s_ctrl)
{
	SET_T4KB3_REG(s_ctrl,0x3500,0x01);
}

static void t4kb3_otp_read_disable(struct msm_sensor_ctrl_t *s_ctrl)
{
	SET_T4KB3_REG(s_ctrl,0x3500,0x00);
}

static void t4kb3_otp_read_delay(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t value = 0;
	GET_T4KB3_REG(s_ctrl,0x3500,value);
	while((value&0x80)==0x80)
		msleep(10);
}

static int32_t t4kb3_otp_read_page(struct msm_sensor_ctrl_t *s_ctrl, uint8_t otp_data[3][64], uint16_t page)
{
    uint16_t i = 0;
    int32_t rc = 0;
    uint8_t *buff = NULL;
    uint32_t num_byte = 0;
	
    num_byte = T4KB3_OTP_DATA_END_ADDR - T4KB3_OTP_DATA_BEGIN_ADDR + 1;
    buff = kzalloc(num_byte, GFP_KERNEL);
    if (buff == NULL)
    {
        pr_err("%s:%d no memory\n", __func__, __LINE__);
        return -ENOMEM;
    }
    
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
                    s_ctrl->sensor_i2c_client,
                    T4KB3_OTP_DATA_BEGIN_ADDR,
                    buff, num_byte);
	    for (i = 0; i < num_byte; i++) 
	    {
	        otp_data[page][i] = buff[i];
	    }
    
    kfree(buff);

    return 0;
}

static int8_t t4kb3_get_AF_info(t4kb3_opt_info *t4kb3_otp,uint8_t  af[24])
{
	uint8_t  sum=0,sum1=0;
	//infinity 
	//select active page
	pr_err(" %s %d: E \n",__func__,__LINE__);
	if((t4kb3_otp->active_place[3]&0x04) == 0x04)
	{
			//check sum
			sum=af[16]+af[17];
			if(af[19]==(sum&0xff))
				{
					pr_err(" %s %d: AF infinity check sum success \n",__func__,__LINE__);
					t4kb3_otp->AF_infinity[0]=af[16];
					t4kb3_otp->AF_infinity[1]=af[17];
				}
			else
				{
					pr_err("zouyu AF infinity %d check sum error!\n",t4kb3_otp->active_place[3]);
					return T4KB3_ERR;
				}
	}
	else if((t4kb3_otp->active_place[3]&0x02) == 0x02)
	{
			//check sum
			sum=af[8]+af[9];
			if(af[11]==(sum&0xff))
				{
					pr_err(" %s %d: AF  infinity check sum success \n",__func__,__LINE__);
					t4kb3_otp->AF_infinity[0]=af[8];
					t4kb3_otp->AF_infinity[1]=af[9];
				}
			else
				{
					pr_err("zouyu AF infinity %d check sum error!\n",t4kb3_otp->active_place[3]);
					return T4KB3_ERR;
				}
	}
	else if((t4kb3_otp->active_place[3]&0x01) == 0x01)
	{
			//check sum
			sum=af[0]+af[1];
			if(af[3]==(sum&0xff))
				{
					pr_err(" %s %d: AF infinity check sum success \n",__func__,__LINE__);
					t4kb3_otp->AF_infinity[0]=af[0];
					t4kb3_otp->AF_infinity[1]=af[1];
				}
			else
				{
					pr_err("zouyu AF infinity %d check sum error!\n",t4kb3_otp->active_place[3]);
					return T4KB3_ERR;
				}

	}

	else
	{
		pr_err("zouyu no AF infinity OTP info!\n");
		return T4KB3_ERR;
	}
		//macro
		//select active page
		if((t4kb3_otp->active_place[2]&0x04)==0x04)
		{
				//check sum
				sum1=af[20]+af[21];
				if(af[23]==(sum1&0xff))
					{
						pr_err(" %s %d: AF macro check sum success \n",__func__,__LINE__);
						t4kb3_otp->AF_macro[0]=af[20];
						t4kb3_otp->AF_macro[1]=af[21];
					}
				else
					{
						pr_err("zouyu AF macro %d check sum error!\n",t4kb3_otp->active_place[2]);
						return T4KB3_ERR;
					}
		}
		else if((t4kb3_otp->active_place[2]&0x02)==0x02)
		{
			sum1=af[12]+af[13];
			//check sum
			if(af[15]==(sum1&0xff))
				{
					pr_err(" %s %d: AF macro check sum success \n",__func__,__LINE__);
					t4kb3_otp->AF_macro[0]=af[12];
					t4kb3_otp->AF_macro[1]=af[13];
				}
			else
				{
					pr_err("AF macro %d check sum error!\n",t4kb3_otp->active_place[2]);
					return T4KB3_ERR;
				}
		}
		else if((t4kb3_otp->active_place[2]&0x01)==0x01)
		{
			//check sum
			sum1=af[4]+af[5];
			if(af[7]==(sum1&0xff))
				{
					pr_err(" %s %d: AF macro check sum success \n",__func__,__LINE__);
					t4kb3_otp->AF_macro[0]=af[4];
					t4kb3_otp->AF_macro[1]=af[5];
				}
			else
				{
					pr_err("AF macro %d check sum error!\n",t4kb3_otp->active_place[2]);
					return T4KB3_ERR;
				}
		
		}
		else
		{
			pr_err("zouyu no AF macro OTP info\n");
			return T4KB3_ERR;
		}
		pr_err(" %s %d: X \n",__func__,__LINE__);
	return 0;
}


static uint16_t t4kb3_get_module_info(t4kb3_opt_info *t4kb3_otp,uint8_t module[32])
{
	uint8_t sum=0;
	uint16_t i;
	//select active page
	if((t4kb3_otp->active_place[1]&0x02)==0x02)
	{
		//check sum
		for(i=0;i<15;i++)
			sum=sum+module[i+16];
		if(module[31]==(sum&0xff))
			{
				for(i=0;i<15;i++)
				{	
					t4kb3_otp->module_info[i]=module[i+16];
				}
				t4kb3_golden_r=(((uint16_t)module[25]<<8)|(uint16_t)module[26]);
				t4kb3_golden_g=(((uint16_t)module[27]<<8)|(uint16_t)module[28]);
				t4kb3_golden_b=(((uint16_t)module[29]<<8)|(uint16_t)module[30]);
			}
		else
			{
				pr_err("zouyu module info %d check sum error!\n",t4kb3_otp->active_place[1]);
				return T4KB3_ERR;
			}
	}		
	else if((t4kb3_otp->active_place[1]&0x01)==0x01)
	{
		//check sum
		for(i=0;i<15;i++)
			sum=sum+module[i];
		if(module[15]==(sum&0xff))
			{
				for(i=0;i<15;i++)
				{
					t4kb3_otp->module_info[i]=module[i];
				}
				t4kb3_golden_r=(((uint16_t)module[9]<<8)|(uint16_t)module[10]);
				t4kb3_golden_g=(((uint16_t)module[11]<<8)|(uint16_t)module[12]);
				t4kb3_golden_b=(((uint16_t)module[13]<<8)|(uint16_t)module[14]);
			}
		else
			{
				pr_err("zouyu module info %d check sum error!\n",t4kb3_otp->active_place[1]);
				return T4KB3_ERR;
			}
	}
	

	else
	{
		pr_err("zouyu get module info failed!\n");
		return T4KB3_ERR;
	}
	return 0;
}

static uint16_t t4kb3_get_awb_info(t4kb3_opt_info *t4kb3_otp,uint8_t awb[32])
{
	uint16_t i;
	uint8_t sum=0;
	//select active place
	pr_err(" %s %d: E \n",__func__,__LINE__);
	if((t4kb3_otp->active_place[4]&0x04)==0x04)
	{
		//check sum
		for(i=0;i<6;i++)
			sum=sum+awb[i+16];
		if((sum&0xff)==awb[22])
			{
			pr_err(" %s %d: AWB check sum success \n",__func__,__LINE__);
				for(i=0;i<6;i++)
					t4kb3_otp->awb[i]=awb[i+16];
			}
		else
			{
				pr_err("zouyu awb %d check sum error!\n",t4kb3_otp->active_place[4]);
				return T4KB3_ERR;
			}
	}
	else if((t4kb3_otp->active_place[4]&0x02)==0x02)
	{
		//check sum
		for(i=0;i<6;i++)
			sum=sum+awb[i+8];
		if((sum&0xff)==awb[14])
			{
				pr_err(" %s %d: AWB check sum success \n",__func__,__LINE__);
				for(i=0;i<6;i++)
					t4kb3_otp->awb[i]=awb[i+8];
			}
		else
			{
				pr_err("zouyu awb %d check sum error!\n",t4kb3_otp->active_place[4]);
				return T4KB3_ERR;
			}
	}
	else if((t4kb3_otp->active_place[4]&0x01)==0x01)
	{
		//check sum
		for(i=0;i<6;i++)
			sum=sum+awb[i];
		if((sum&0xff)==awb[6])
			{
				pr_err(" %s %d: AWB check sum success \n",__func__,__LINE__);
				for(i=0;i<6;i++)
					t4kb3_otp->awb[i]=awb[i];
			}
		else
			{
				pr_err("zouyu awb %d check sum error!\n",t4kb3_otp->active_place[4]);
				return T4KB3_ERR;
			}
	}
	else
	{
		pr_err("zouyu get awb info failed!\n");
		return T4KB3_ERR;
	}
	return 0;
}

static uint16_t t4kb3_get_lsc_info(struct msm_sensor_ctrl_t *s_ctrl,t4kb3_opt_info *t4kb3_otp)
{
	if((t4kb3_otp->active_place[0]&0x02)==0x02)
	{
		SET_T4KB3_REG(s_ctrl,0x3551,0x85);//load lsc_#2
		mdelay(1);
		pr_err("zouyu lsc_#2 is used.\n");
		return 2;
	}
	else if((t4kb3_otp->active_place[0]&0x01)==0x01)
	{
		pr_err("zouyu lsc_#1 is used.\n");
		return 1;
	}
	else
	{
		pr_err("zouyu! LSC error!\n");
		return T4KB3_ERR;
	}
	return 0;
}
static void  t4kb3_otp_read(struct msm_sensor_ctrl_t *s_ctrl, t4kb3_opt_info *t4kb3_otp)
{
	uint8_t a[3][64];
	uint16_t i,j;
	uint8_t  af[24];
	uint8_t module[32];
	uint8_t awb[32];
	//uint16_t lsc[2][64];

	pr_err(" %s %d: E \n",__func__,__LINE__);
	memset(t4kb3_otp,0,sizeof(t4kb3_opt_info));
	
	//read page0-page6 otp data
	for(i=0;i<3;i++)
	{
		t4kb3_otp_read_enable(s_ctrl);
		t4kb3_otp_set_page(s_ctrl,i);
		t4kb3_otp_access(s_ctrl);
		t4kb3_otp_read_delay(s_ctrl);
		t4kb3_otp_read_page(s_ctrl,a,i);
		t4kb3_otp_read_disable(s_ctrl);
	}
	for(j=0;j<3;j++)
		{
		for(i=0;i<64;i++)
			{
				pr_err("zouyu page %d data %d = 0x%x\n",j,i,a[j][i]);
			}
		}
	//get active page info
	for(i=0;i<5;i++)
		t4kb3_otp->active_place[i]=a[0][i]|a[0][i+32];
	//get AF info
	for(i=0;i<24;i++)
		af[i]=a[0][8+i]|a[0][i+40];
	if(t4kb3_get_AF_info(t4kb3_otp,af)==T4KB3_ERR)
		t4kb3_otp->er.af_err++;
	//get module info
	for(i=0;i<32;i++)
		module[i]=a[1][i]|a[1][i+32];
	if(t4kb3_get_module_info(t4kb3_otp,module)==T4KB3_ERR)
		t4kb3_otp->er.module_err++;
	//get awb info
	for(i=0;i<32;i++)
		awb[i]=a[2][i]|a[2][i+32];
	if(t4kb3_get_awb_info(t4kb3_otp,awb)==T4KB3_ERR)
		t4kb3_otp->er.awb_err++;
	//get lsc_#3 info
	if(t4kb3_get_lsc_info(s_ctrl,t4kb3_otp)==T4KB3_ERR)
	{
		t4kb3_otp->er.lsc_err++;
		pr_err("LSC update failed\n");
	}
   pr_err(" %s %d: X \n",__func__,__LINE__);	
}

#if 1
static void t4kb3_awb_update(struct msm_sensor_ctrl_t *s_ctrl,t4kb3_opt_info *t4kb3_otp)
{
	uint16_t  r,g,b;
	uint16_t r_gain,b_gain,g_gain;
	uint16_t AWB_R = 0;
	uint16_t AWB_B = 0;
	uint16_t AWB_G = 0;

	pr_err(" %s %d: E \n",__func__,__LINE__);
	if((t4kb3_otp->er.awb_err == 0) && (t4kb3_otp->er.module_err == 0))
	{
		pr_err("t4kb3_golden_r %d \n",t4kb3_golden_r);
		pr_err("t4kb3_golden_g %d \n",t4kb3_golden_g);
		pr_err("t4kb3_golden_b %d \n",t4kb3_golden_b);
		
		r=(((uint16_t)t4kb3_otp->awb[0]<<8))|((uint16_t)t4kb3_otp->awb[1]);
		g=((uint16_t)(t4kb3_otp->awb[2]<<8))|((uint16_t)t4kb3_otp->awb[3]);
		b=((uint16_t)(t4kb3_otp->awb[4]<<8))|((uint16_t)t4kb3_otp->awb[5]);

		pr_err("T4KB3_AVE_R %d\n",r);
		pr_err("T4KB3_AVE_G %d\n",g);
		pr_err("T4KB3_AVE_B %d\n",b);

#if 1		
		//calculate gain
		AWB_R = (t4kb3_golden_r * g)/ ( r * t4kb3_golden_g);
		AWB_B =(t4kb3_golden_b * g )/ (b * t4kb3_golden_g);
#endif
		if(AWB_R<AWB_B)
		{
			//AWB_G=1/AWB_R;
			AWB_G = ( r * t4kb3_golden_g) /(t4kb3_golden_r * g);
		//	r_gain=(AWB_R*AWB_G*256);
		//	b_gain=(AWB_B*AWB_G*256);
		//	g_gain=(AWB_G*256);		
			r_gain=(((t4kb3_golden_r * g)/ ( r * t4kb3_golden_g))*(( r * t4kb3_golden_g) /(t4kb3_golden_r * g))*256);
			b_gain=(((t4kb3_golden_b * g )/ (b * t4kb3_golden_g))*(( r * t4kb3_golden_g) /(t4kb3_golden_r * g))*256);
			g_gain=((( r * t4kb3_golden_g) /(t4kb3_golden_r * g))*256);
		}
		else
		{
			//AWB_G=1/AWB_B;
			AWB_G= (b * t4kb3_golden_g)/(t4kb3_golden_b * g);
		//	r_gain=(AWB_R*AWB_G*256);
		//	b_gain=(AWB_B*AWB_G*256);
		//	g_gain=(AWB_G*256);		
			r_gain=(((t4kb3_golden_r * g)/ ( r * t4kb3_golden_g))*((b * t4kb3_golden_g)/(t4kb3_golden_b * g))*256);
			b_gain=(((t4kb3_golden_b * g )/ (b * t4kb3_golden_g))*((b * t4kb3_golden_g)/(t4kb3_golden_b * g))*256);
			g_gain=(((b * t4kb3_golden_g)/(t4kb3_golden_b * g))*256);			
		}
		
		if(AWB_G <= 1)
		{
			AWB_G=1;
			//	r_gain=(AWB_R*AWB_G*256);
			//	b_gain=(AWB_B*AWB_G*256);
			//	g_gain=(AWB_G*256);
			r_gain=(((t4kb3_golden_r * g)/ ( r * t4kb3_golden_g))* 1 *256);
			b_gain=(((t4kb3_golden_b * g )/ (b * t4kb3_golden_g))* 1 *256);
			g_gain=(1 *256);				
		}


		pr_err(" r_gain %d\n", r_gain);
		pr_err("b_gain %d\n", b_gain);
		pr_err("g_gain %d\n", g_gain);
	//	r_gain=(AWB_R*AWB_G*256);
	//	b_gain=(AWB_B*AWB_G*256);
	//	g_gain=(AWB_G*256);
		//write gain
		SET_T4KB3_REG(s_ctrl,0x020e,g_gain>>8);
		SET_T4KB3_REG(s_ctrl,0x020f,g_gain&0xff);
		
		SET_T4KB3_REG(s_ctrl,0x0210,r_gain>>8);
		SET_T4KB3_REG(s_ctrl,0x0211,r_gain&0xff);

		SET_T4KB3_REG(s_ctrl,0x0212,b_gain>>8);
		SET_T4KB3_REG(s_ctrl,0x0213,b_gain&0xff);

		SET_T4KB3_REG(s_ctrl,0x0214,g_gain>>8);
		SET_T4KB3_REG(s_ctrl,0x0215,g_gain&0xff);
	}
	else
	{
		pr_err("zouyu! awb update failed!\n");
	}

	pr_err(" %s %d: X \n",__func__,__LINE__);
}
#endif
static void t4kb3_AF_update(struct msm_sensor_ctrl_t *s_ctrl,t4kb3_opt_info *t4kb3_otp)
{
      pr_err(" %s %d: E \n",__func__,__LINE__);
	  
	if(t4kb3_otp->active_place[0] == 0x00)
	{
		t4kb3_af_macro_value = t4kb3_af_macro_pos;
		t4kb3_af_inifity_value= t4kb3_af_inifity_pos;
	}
	else if(t4kb3_otp->er.af_err == 0x00)
	{
		t4kb3_af_macro_value = (((uint16_t)t4kb3_otp->AF_macro[0])<<8)|((uint16_t)t4kb3_otp->AF_macro[1]);
		t4kb3_af_inifity_value = (((uint16_t)t4kb3_otp->AF_infinity[0])<<8)|((uint16_t)t4kb3_otp->AF_infinity[1]);
		t4kb3_af_otp_status = 1;
	}
	else
		pr_err("zouyu! AF update failed!\n");
	
	pr_err("t4kb3_AF_update  macro=%d,infinity=%d\n",t4kb3_af_macro_value,t4kb3_af_inifity_value);

	pr_err(" %s %d: X \n",__func__,__LINE__);
}


 
 void t4kb3_update_otp_para(struct msm_sensor_ctrl_t *s_ctrl)
{
	t4kb3_opt_info *t4kb3_otp=NULL;
	
       t4kb3_af_otp_status = 0;
	
	//t4kb3_otp = (t4kb3_opt_info *)malloc(sizeof(t4kb3_opt_info));
       t4kb3_otp = kzalloc(sizeof (t4kb3_opt_info), GFP_KERNEL); 
          
	pr_err(" %s %d: E \n",__func__,__LINE__);
	
    if(strcmp(s_ctrl->sensordata->sensor_name, "t4kb3"))
    {
        pr_err("zouyu -result =:%d\n", strcmp(s_ctrl->sensordata->sensor_name, "t4kb3"));    
    }
else
	{
		pr_err("zouyu t4kb3_update_otp_para begin\n");
		t4kb3_otp_read(s_ctrl,t4kb3_otp);
		//update awb
		t4kb3_awb_update(s_ctrl,t4kb3_otp);
		//update AF
		t4kb3_AF_update(s_ctrl,t4kb3_otp);
		pr_err("zouyu t4kb3_update_otp_para finish \n");
	}

	kfree(t4kb3_otp);

      pr_err(" %s %d: X \n",__func__,__LINE__);	
}

