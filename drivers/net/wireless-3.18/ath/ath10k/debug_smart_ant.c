/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifdef CONFIG_ATH10K_SMART_ANTENNA
#include "core.h"
#include "wmi-ops.h"
#include "debug.h"
#include "smart_ant.h"

static ssize_t ath10k_write_sa_enable_ops(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret;
	u8 enable;

	if (!ath10k_smart_ant_enabled(ar))
		return -ENOTSUPP;

	if (kstrtou8_from_user(user_buf, count, 0, &enable))
		return -EINVAL;

	if (ar->smart_ant_info.enabled == enable)
		return count;

	mutex_lock(&ar->conf_mutex);
	if (enable) {
		ret = ath10k_wmi_pdev_enable_smart_ant(ar,
				WMI_SMART_ANT_MODE_PARALLEL,
				ATH10K_SMART_ANT_DEFAULT_ANT,
				ATH10K_SMART_ANT_DEFAULT_ANT);
		if (ret)
			goto exit;

		ret = ath10k_wmi_pdev_pktlog_enable(ar,
				ar->debug.pktlog_filter |
				ATH10K_PKTLOG_SMART_ANT);
		if (ret)
			goto exit;
		ar->debug.pktlog_filter |= ATH10K_PKTLOG_SMART_ANT;
	} else {
		ret = ath10k_wmi_pdev_disable_smart_ant(ar,
				WMI_SMART_ANT_MODE_PARALLEL, 0, 0);
		if (ret)
			goto exit;

		ar->debug.pktlog_filter &= ~ATH10K_PKTLOG_SMART_ANT;
		if (ar->debug.pktlog_filter) {
			ath10k_wmi_pdev_pktlog_enable(ar,
					      ar->debug.pktlog_filter);
		} else {
			ath10k_wmi_pdev_pktlog_disable(ar);
		}
	}
	ar->smart_ant_info.enabled = enable;

	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "Smart antenna %s\n",
		   enable ? "enabled" : "disabled");
exit:
	mutex_unlock(&ar->conf_mutex);
	if (ret)
		return ret;

	return count;
}

static ssize_t ath10k_read_sa_enable_ops(struct file *file, char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[4];
	int len = 0;

	if (!ath10k_smart_ant_enabled(ar))
		return -ENOTSUPP;

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->smart_ant_info.enabled);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations fops_sa_enable_ops = {
	.write = ath10k_write_sa_enable_ops,
	.read = ath10k_read_sa_enable_ops,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_sa_tx_ant_ops(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 ants[WMI_SMART_ANT_RATE_SERIES_MAX], txant;
	u8 mac_addr[ETH_ALEN];
	struct ieee80211_sta *sta;
	struct ath10k_sta *arsta;
	int ret, i, vdev_id, len;
	char *token, *sptr;
	char buf[64];

	if (!ath10k_smart_ant_enabled(ar))
		return -ENOTSUPP;

	len = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;

	buf[len] = '\0';
	sptr = buf;
	for (i = 0; i < ETH_ALEN - 1; i++) {
		token = strsep(&sptr, ":");
		if (!token)
			return -EINVAL;

		if (kstrtou8(token, 16, &mac_addr[i]))
			return -EINVAL;
	}

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;

	if (kstrtou8(token, 16, &mac_addr[i]))
		return -EINVAL;

	if (kstrtou32(sptr, 0, &txant))
		return -EINVAL;

	if (txant > ar->supp_tx_chainmask) {
		ath10k_err(ar, "Invalid tx antenna config\n");
		return -EINVAL;
	}

	rcu_read_lock();

	sta = ieee80211_find_sta_by_ifaddr(ar->hw, mac_addr, NULL);
	if (!sta) {
		ath10k_err(ar, "Sta entry not found\n");
		rcu_read_unlock();
		return -EINVAL;
	}

	arsta = (struct ath10k_sta *)sta->drv_priv;
	vdev_id = arsta->arvif->vdev_id;

	rcu_read_unlock();

	for (i = 0; i < WMI_SMART_ANT_RATE_SERIES_MAX; i++)
		ants[i] = txant;

	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "Smart antenna set tx antenna to %d\n",
		   txant);
	mutex_lock(&ar->conf_mutex);
	ret = ath10k_wmi_peer_set_smart_tx_ant(ar, vdev_id, mac_addr,
			ants, WMI_SMART_ANT_RATE_SERIES_MAX);
	mutex_unlock(&ar->conf_mutex);

	if (!ret)
		ret = count;

	return ret;
}

static const struct file_operations fops_sa_tx_ant_ops = {
	.write = ath10k_write_sa_tx_ant_ops,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_read_sa_rx_ant_ops(struct file *file, char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	char buf[4];
	struct ath10k *ar = file->private_data;
	int len = 0;

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->smart_ant_info.rx_antenna);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t ath10k_write_sa_rx_ant_ops(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 rxant;
	int ret;

	if (!ath10k_smart_ant_enabled(ar))
		return -ENOTSUPP;

	if (kstrtou8_from_user(user_buf, count, 0, &rxant))
		return -EINVAL;

	if (rxant > ar->supp_rx_chainmask) {
		ath10k_err(ar, "Invalid Rx antenna config\n");
		return -EINVAL;
	}

	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		   "Setting Rx antenna to %d\n", rxant);

	mutex_lock(&ar->conf_mutex);
	ret = ath10k_wmi_pdev_set_rx_ant(ar, rxant);
	mutex_unlock(&ar->conf_mutex);

	if (!ret)
		ret = count;

	return ret;
}

static const struct file_operations fops_sa_rx_ant_ops = {
	.read = ath10k_read_sa_rx_ant_ops,
	.write = ath10k_write_sa_rx_ant_ops,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_sa_train_info_ops(struct file *file,
					      const char __user *user_buf,
					      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 mac_addr[ETH_ALEN];
	struct ieee80211_sta *sta;
	struct ath10k_sta *arsta;
	struct wmi_peer_sant_set_train_arg arg;
	int ret, i, vdev_id, len;
	char *token, *sptr;
	char buf[128];

	if (!ath10k_smart_ant_enabled(ar))
		return -ENOTSUPP;

	len = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;

	buf[len] = '\0';
	sptr = buf;
	for (i = 0; i < ETH_ALEN - 1; i++) {
		token = strsep(&sptr, ":");
		if (!token)
			return -EINVAL;

		if (kstrtou8(token, 16, &mac_addr[i]))
			return -EINVAL;
	}

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;

	if (kstrtou8(token, 16, &mac_addr[i]))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 16, &arg.rates[0]))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &arg.antennas[0]))
		return -EINVAL;

	if (kstrtou32(sptr, 0, &arg.num_pkts))
		return -EINVAL;

	for (i = 0; i < WMI_SMART_ANT_RATE_SERIES_MAX; i++) {
		arg.rates[i] = arg.rates[0];
		arg.antennas[i] = arg.antennas[0];
	}

	if (arg.antennas[0] > ar->supp_tx_chainmask) {
		ath10k_err(ar, "Invalid tx ant for trianing\n");
		return -EINVAL;
	}
	/* TODO: Validate rate input */
	rcu_read_lock();

	sta = ieee80211_find_sta_by_ifaddr(ar->hw, mac_addr, NULL);
	if (!sta) {
		ath10k_err(ar, "Sta entry not found\n");
		rcu_read_unlock();
		return -EINVAL;
	}

	arsta = (struct ath10k_sta *)sta->drv_priv;
	vdev_id = arsta->arvif->vdev_id;

	rcu_read_unlock();

	for (i = 0; i < WMI_SMART_ANT_RATE_SERIES_MAX; i++) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "rate[%d] 0x%x antenna[%d] %d\n",
			   i, arg.rates[i], i, arg.antennas[i]);
	}

	mutex_lock(&ar->conf_mutex);
	ret = ath10k_wmi_peer_set_smart_ant_train_info(ar, vdev_id,
						       mac_addr, &arg);
	mutex_unlock(&ar->conf_mutex);
	if (!ret)
		ret = count;

	return ret;
}

static const struct file_operations fops_sa_train_info_ops = {
	.write = ath10k_write_sa_train_info_ops,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

void ath10k_smart_ant_debugfs_init(struct ath10k *ar)
{
	debugfs_create_file("smart_ant_enable", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_sa_enable_ops);

	debugfs_create_file("smart_ant_tx_ant", S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_sa_tx_ant_ops);

	debugfs_create_file("smart_ant_rx_ant", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_sa_rx_ant_ops);

	debugfs_create_file("smart_ant_train_info", S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_sa_train_info_ops);
}
#endif
