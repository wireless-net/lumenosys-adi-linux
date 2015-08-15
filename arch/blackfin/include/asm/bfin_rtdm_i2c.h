/* bfin_rtdm_i2c.h --- 
 * 
 * Filename: bfin_rtdm_i2c.h
 * Description: 
 * Author: Devin Butterfield
 * Maintainer: 
 * Created: Sat Feb  8 22:17:46 2014 (-0800)
 * Version: 
 * Last-Updated: Fri Mar 21 14:26:03 2014 (-0700)
 *           By: Devin Butterfield
 *     Update #: 11
 * URL: 
 * Keywords: 
 * Compatibility: 
 * 
 */

/* Commentary: 
 * 
 * 
 * 
 */

/* Change Log:
 * 
 * 
 */

/* *This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 3, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth
 * Floor, Boston, MA 02110-1301, USA.
 */

/* Code: */

#ifndef __BFIN_RTDM_I2C_H
#define __BFIN_RTDM_I2C_H

#include <linux/ioctl.h>
#include <linux/i2c.h>

/* reusing some of the structs from the linux i2c-dev interface */
#include <linux/i2c-dev.h>

#include <rtdm/rtdm.h>

/* structure for IOCTL calls */

struct bfin_rtdm_i2c_ioctl_data {
  unsigned short addr;
  unsigned int flags;
  struct i2c_smbus_ioctl_data smbus_data;
};

#endif//__BFIN_RTDM_I2C_H

/* bfin_rtdm_i2c.h ends here */
