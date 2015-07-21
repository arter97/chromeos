/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _DRV_CROS_EC_DEV_H_
#define _DRV_CROS_EC_DEV_H_


/* sysfs stuff */
extern struct attribute_group cros_ec_attr_group;
extern struct attribute_group cros_ec_lightbar_attr_group;
extern struct attribute_group cros_ec_pd_attr_group;

/* lightbar utilities */
extern bool ec_has_lightbar(struct cros_ec_dev *ec);
extern int lb_manual_suspend_ctrl(struct cros_ec_dev *ec, uint8_t enable);
extern int lb_suspend(struct cros_ec_dev *ec);
extern int lb_resume(struct cros_ec_dev *ec);

#endif	/* _DRV_CROS_EC_DEV_H_ */
