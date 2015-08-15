/* ad7091R2.h --- 
 * 
 * Filename: ad7091R2.h
 * Description: 
 * Author: Devin Butterfield
 * Maintainer: 
 * Created: Sat Jan  3 19:06:03 2015 (-0800)
 * Version: 
 * Last-Updated: Sun Jan  4 20:45:41 2015 (-0800)
 *           By: Devin Butterfield
 *     Update #: 4
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

#ifndef IIO_ADC_AD7091R_H_
#define IIO_ADC_AD7091R_H_

/**
 * struct ad7091R_platform_data - platform/board specifc information
 * @config              device configuration bits
 * @gpio_convst:	number of gpio connected to the CONVST pin
 */

struct ad7091R2_platform_data {
	uint16_t			mode;
	unsigned			gpio_convst;
};

#endif/*IIO_ADC_AD7091R_H_*/

/* ad7091R2.h ends here */
