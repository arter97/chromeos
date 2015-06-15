#include <linux/if_ether.h>
#include <net/cfg80211.h>
#include <linux/errqueue.h>
#include <generated/utsrelease.h>
/* ipv6_addr_is_multicast moved - include old header */
#include <net/addrconf.h>

/* common backward compat code */

#define BACKPORTS_GIT_TRACKED "chromium:" UTS_RELEASE
#define BACKPORTS_BUILD_TSTAMP __DATE__ " " __TIME__

static inline void netdev_attach_ops(struct net_device *dev,
				     const struct net_device_ops *ops)
{
	dev->netdev_ops = ops;
}

#define WIPHY_FLAG_HAS_CHANNEL_SWITCH 0

#define WIPHY_PARAM_DYN_ACK		(1 << 5)

#define mc_addr(ha)	(ha)->addr

#define NL80211_FEATURE_STATIC_SMPS		(1 << 24)
#define NL80211_FEATURE_DYNAMIC_SMPS		(1 << 25)
#define NL80211_FEATURE_SUPPORTS_WMM_ADMISSION	(1 << 26)
/* cannot be supported on this kernel */
#define NL80211_FEATURE_TDLS_CHANNEL_SWITCH	0

static inline void
cfg80211_ch_switch_started_notify(struct net_device *dev,
				  struct cfg80211_chan_def *chandef)
{
}

/* cfg80211 version specific backward compat code follows */
#define CFG80211_VERSION KERNEL_VERSION(3,8,0)

#if CFG80211_VERSION < KERNEL_VERSION(3,9,0)
struct wiphy_wowlan_tcp_support {
	const struct nl80211_wowlan_tcp_data_token_feature *tok;
	u32 data_payload_max;
	u32 data_interval_max;
	u32 wake_payload_max;
	bool seq;
};

struct cfg80211_wowlan_tcp {
	struct socket *sock;
	__be32 src, dst;
	u16 src_port, dst_port;
	u8 dst_mac[ETH_ALEN];
	int payload_len;
	const u8 *payload;
	struct nl80211_wowlan_tcp_data_seq payload_seq;
	u32 data_interval;
	u32 wake_len;
	const u8 *wake_data, *wake_mask;
	u32 tokens_size;
	/* must be last, variable member */
	struct nl80211_wowlan_tcp_data_token payload_tok;
};

#endif /* CFG80211_VERSION < KERNEL_VERSION(3,9,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,10,0)
static inline bool
ieee80211_operating_class_to_band(u8 operating_class,
				  enum ieee80211_band *band)
{
	switch (operating_class) {
	case 112:
	case 115 ... 127:
		*band = IEEE80211_BAND_5GHZ;
		return true;
	case 81:
	case 82:
	case 83:
	case 84:
		*band = IEEE80211_BAND_2GHZ;
		return true;
	case 180:
		*band = IEEE80211_BAND_60GHZ;
		return true;
	}

	/* stupid compiler */
	*band = IEEE80211_BAND_2GHZ;

	return false;
}

#define NL80211_FEATURE_USERSPACE_MPM 0
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,10,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
#define IEEE80211_RADIOTAP_MCS_HAVE_STBC 0
#endif

#if CFG80211_VERSION < KERNEL_VERSION(3,11,0)
#define IEEE80211_MAX_CHAINS 4

#define MONITOR_FLAG_ACTIVE 0

static inline void cfg80211_rx_unprot_mlme_mgmt(struct net_device *dev,
						void *data, int len)
{
	struct ieee80211_hdr *hdr = data;

	if (ieee80211_is_deauth(hdr->frame_control))
		cfg80211_send_unprot_deauth(dev, data, len);
	else
		cfg80211_send_unprot_disassoc(dev, data, len);
}

static inline void cfg80211_tx_mlme_mgmt(struct net_device *dev,
					 void *data, int len)
{
	struct ieee80211_hdr *hdr = data;

	if (ieee80211_is_deauth(hdr->frame_control))
		cfg80211_send_deauth(dev, data, len);
	else
		cfg80211_send_disassoc(dev, data, len);
}

static inline void cfg80211_rx_mlme_mgmt(struct net_device *dev,
					 void *data, int len)
{
	struct ieee80211_hdr *hdr = data;

	if (ieee80211_is_auth(hdr->frame_control))
		cfg80211_send_rx_auth(dev, data, len);
	else if (ieee80211_is_deauth(hdr->frame_control))
		cfg80211_send_deauth(dev, data, len);
	else
		cfg80211_send_disassoc(dev, data, len);
}

static inline void cfg80211_assoc_timeout(struct net_device *dev,
					  struct cfg80211_bss *bss)
{
	cfg80211_send_assoc_timeout(dev, bss->bssid);
}

static inline void cfg80211_auth_timeout(struct net_device *dev,
					 const u8 *bssid)
{
	cfg80211_send_auth_timeout(dev, bssid);
}

static inline void cfg80211_rx_assoc_resp(struct net_device *dev,
					  struct cfg80211_bss *bss,
					  void *data, int len,
					  int uapsd_queues)
{
	cfg80211_send_rx_assoc(dev, bss, data, len);
}

static inline enum ieee80211_rate_flags
ieee80211_chandef_rate_flags(struct cfg80211_chan_def *chandef)
{
	return 0;
}

#define IEEE80211_RADIOTAP_MCS_STBC_SHIFT	5

/* on older versions this is safe - no RTNL use there */
#define cfg80211_sched_scan_stopped_rtnl cfg80211_sched_scan_stopped
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,11,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0)
#define IEEE80211_CHAN_HALF 0
#define IEEE80211_CHAN_QUARTER 0
#endif

#if CFG80211_VERSION < KERNEL_VERSION(3,12,0)
static inline int
ieee80211_chandef_max_power(struct cfg80211_chan_def *chandef)
{
	return chandef->chan->max_power;
}

static inline struct cfg80211_bss * __must_check
cfg80211_inform_bss_width_frame(struct wiphy *wiphy,
				struct ieee80211_channel *channel,
				enum nl80211_bss_scan_width scan_width,
				struct ieee80211_mgmt *mgmt, size_t len,
				s32 signal, gfp_t gfp)
{
	return cfg80211_inform_bss_frame(wiphy, channel, mgmt,
					 len, signal, gfp);
}

static inline enum nl80211_bss_scan_width
cfg80211_chandef_to_scan_width(const struct cfg80211_chan_def *chandef)
{
	return NL80211_BSS_CHAN_WIDTH_20;
}

static inline bool
iwl7000_cfg80211_rx_mgmt(struct wireless_dev *wdev, int freq, int sig_dbm,
			 const u8 *buf, size_t len, u32 flags)
{
	return cfg80211_rx_mgmt(wdev, freq, sig_dbm, buf, len, GFP_ATOMIC);
}
#define cfg80211_rx_mgmt iwl7000_cfg80211_rx_mgmt

struct cfg80211_csa_settings {
	struct cfg80211_chan_def chandef;
	struct cfg80211_beacon_data beacon_csa;
	u16 counter_offset_beacon, counter_offset_presp;
	struct cfg80211_beacon_data beacon_after;
	bool radar_required;
	bool block_tx;
	u8 count;
};

static inline u32
ieee80211_mandatory_rates(struct ieee80211_supported_band *sband)
{
	struct ieee80211_rate *bitrates;
	u32 mandatory_rates = 0;
	enum ieee80211_rate_flags mandatory_flag;
	int i;

	if (WARN_ON(!sband))
		return 1;

	if (sband->band == IEEE80211_BAND_2GHZ)
		mandatory_flag = IEEE80211_RATE_MANDATORY_B;
	else
		mandatory_flag = IEEE80211_RATE_MANDATORY_A;

	bitrates = sband->bitrates;
	for (i = 0; i < sband->n_bitrates; i++)
		if (bitrates[i].flags & mandatory_flag)
			mandatory_rates |= BIT(i);
	return mandatory_rates;
}

#define ieee80211_mandatory_rates(sband, width) ieee80211_mandatory_rates(sband)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,12,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,13,0)
static inline int cfg80211_chandef_get_width(const struct cfg80211_chan_def *c)
{
	int width;

	switch (c->width) {
	case NL80211_CHAN_WIDTH_20:
	case NL80211_CHAN_WIDTH_20_NOHT:
		width = 20;
		break;
	case NL80211_CHAN_WIDTH_40:
		width = 40;
		break;
	case NL80211_CHAN_WIDTH_80P80:
	case NL80211_CHAN_WIDTH_80:
		width = 80;
		break;
	case NL80211_CHAN_WIDTH_160:
		width = 160;
		break;
	default:
		WARN_ON_ONCE(1);
		return -1;
	}
	return width;
}

static inline int cfg80211_get_chans_dfs_required(struct wiphy *wiphy,
						  u32 center_freq,
						  u32 bandwidth)
{
	struct ieee80211_channel *c;
	u32 freq, start_freq, end_freq;

	if (bandwidth <= 20) {
		start_freq = center_freq;
		end_freq = center_freq;
	} else {
		start_freq = center_freq - bandwidth/2 + 10;
		end_freq = center_freq + bandwidth/2 - 10;
	}

	for (freq = start_freq; freq <= end_freq; freq += 20) {
		c = ieee80211_get_channel(wiphy, freq);
		if (!c)
			return -EINVAL;

		if (c->flags & IEEE80211_CHAN_RADAR)
			return 1;
	}
	return 0;
}

#define cfg80211_radar_event(...) do { } while (0)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,13,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,14,0)
struct cfg80211_mgmt_tx_params {
	struct ieee80211_channel *chan;
	bool offchan;
	unsigned int wait;
	const u8 *buf;
	size_t len;
	bool no_cck;
	bool dont_wait_for_ack;
};

#define regulatory_flags flags

#define REGULATORY_CUSTOM_REG WIPHY_FLAG_CUSTOM_REGULATORY
#define REGULATORY_DISABLE_BEACON_HINTS WIPHY_FLAG_DISABLE_BEACON_HINTS

#define IEEE80211_CHAN_NO_IR (IEEE80211_CHAN_PASSIVE_SCAN |\
			      IEEE80211_CHAN_NO_IBSS)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,14,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,15,0)
#define IEEE80211_RADIOTAP_CODING_LDPC_USER0	0x1

#define cfg80211_ibss_joined(dev, bssid, chan, gfp) \
	cfg80211_ibss_joined(dev, bssid, gfp)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,15,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,16,0)
#define REGULATORY_ENABLE_RELAX_NO_IR 0

#define cfg80211_reg_can_beacon(wiphy, chandef, iftype) \
	cfg80211_reg_can_beacon(wiphy, chandef)

static inline int
cfg80211_chandef_dfs_required(struct wiphy *wiphy,
			      const struct cfg80211_chan_def *chandef,
			      enum nl80211_iftype iftype)
{
	int width;
	int ret;

	if (WARN_ON(!cfg80211_chandef_valid(chandef)))
		return -EINVAL;

	switch (iftype) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_MESH_POINT:
		width = cfg80211_chandef_get_width(chandef);
		if (width < 0)
			return -EINVAL;

		ret = cfg80211_get_chans_dfs_required(wiphy,
						      chandef->center_freq1,
						      width);
		if (ret < 0)
			return ret;
		else if (ret > 0)
			return BIT(chandef->width);

		if (!chandef->center_freq2)
			return 0;

		ret = cfg80211_get_chans_dfs_required(wiphy,
						      chandef->center_freq2,
						      width);
		if (ret < 0)
			return ret;
		else if (ret > 0)
			return BIT(chandef->width);

		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_P2P_DEVICE:
		break;
	case NL80211_IFTYPE_UNSPECIFIED:
	case NUM_NL80211_IFTYPES:
		WARN_ON(1);
	}

	return 0;
}

static inline int
cfg80211_iter_combinations(struct wiphy *wiphy,
			   const int num_different_channels,
			   const u8 radar_detect,
			   const int iftype_num[NUM_NL80211_IFTYPES],
			   void (*iter)(const struct ieee80211_iface_combination *c,
					void *data),
			   void *data)
{
	int i, j, iftype;
	int num_interfaces = 0;
	u32 used_iftypes = 0;

	for (iftype = 0; iftype < NUM_NL80211_IFTYPES; iftype++) {
		num_interfaces += iftype_num[iftype];
		if (iftype_num[iftype] > 0 &&
		    !(wiphy->software_iftypes & BIT(iftype)))
			used_iftypes |= BIT(iftype);
	}

	for (i = 0; i < wiphy->n_iface_combinations; i++) {
		const struct ieee80211_iface_combination *c;
		struct ieee80211_iface_limit *limits;
		u32 all_iftypes = 0;

		c = &wiphy->iface_combinations[i];

		if (num_interfaces > c->max_interfaces)
			continue;
		if (num_different_channels > c->num_different_channels)
			continue;

		limits = kmemdup(c->limits, sizeof(limits[0]) * c->n_limits,
				 GFP_KERNEL);
		if (!limits)
			return -ENOMEM;

		for (iftype = 0; iftype < NUM_NL80211_IFTYPES; iftype++) {
			if (wiphy->software_iftypes & BIT(iftype))
				continue;
			for (j = 0; j < c->n_limits; j++) {
				all_iftypes |= limits[j].types;
				if (!(limits[j].types & BIT(iftype)))
					continue;
				if (limits[j].max < iftype_num[iftype])
					goto cont;
				limits[j].max -= iftype_num[iftype];
			}
		}

		if (radar_detect)
			goto cont;

		/* Finally check that all iftypes that we're currently
		 * using are actually part of this combination. If they
		 * aren't then we can't use this combination and have
		 * to continue to the next.
		 */
		if ((all_iftypes & used_iftypes) != used_iftypes)
			goto cont;

		/* This combination covered all interface types and
		 * supported the requested numbers, so we're good.
		 */

		(*iter)(c, data);
 cont:
		kfree(limits);
	}

	return 0;
}

static void
cfg80211_iter_sum_ifcombs(const struct ieee80211_iface_combination *c,
			  void *data)
{
	int *num = data;
	(*num)++;
}

static inline int
cfg80211_check_combinations(struct wiphy *wiphy,
			    const int num_different_channels,
			    const u8 radar_detect,
			    const int iftype_num[NUM_NL80211_IFTYPES])
{
	int err, num = 0;

	err = cfg80211_iter_combinations(wiphy, num_different_channels,
					 radar_detect, iftype_num,
					 cfg80211_iter_sum_ifcombs, &num);
	if (err)
		return err;
	if (num == 0)
		return -EBUSY;

	return 0;
}

#define cfg80211_ch_switch_started_notify(dev, chandef, count) \
	cfg80211_ch_switch_started_notify(dev, chandef)
#define const_since_3_16
#else
#define const_since_3_16 const
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,16,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,18,0)
#define NL80211_IFTYPE_OCB 11 /* not used, but code is there */
#define NL80211_FEATURE_MAC_ON_CREATE 0 /* cannot be used */

static inline struct wiphy *
wiphy_new_nm(const struct cfg80211_ops *ops, int sizeof_priv,
	     const char *requested_name)
{
	/* drop the requested name since it's not supported */
	return wiphy_new(ops, sizeof_priv);
}
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,18,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,19,0)
#define NL80211_FEATURE_SCHED_SCAN_RANDOM_MAC_ADDR 0
#define NL80211_FEATURE_SCAN_RANDOM_MAC_ADDR 0
#define NL80211_FEATURE_ND_RANDOM_MAC_ADDR 0
#define NL80211_SCAN_FLAG_RANDOM_ADDR 0

/* LAR related functionality privately backported into mac80211 */
int regulatory_set_wiphy_regd(struct wiphy *wiphy,
			      struct ieee80211_regdomain *rd);
int regulatory_set_wiphy_regd_sync_rtnl(struct wiphy *wiphy,
					struct ieee80211_regdomain *rd);

#define REGULATORY_COUNTRY_IE_IGNORE 0
#define REGULATORY_WIPHY_SELF_MANAGED WIPHY_FLAG_SELF_MANAGED_REG

#define IEEE80211_CHAN_INDOOR_ONLY 0
#define IEEE80211_CHAN_GO_CONCURRENT 0

static inline bool
ieee80211_chandef_to_operating_class(struct cfg80211_chan_def *chandef,
				     u8 *op_class)
{
	u8 vht_opclass;
	u16 freq = chandef->center_freq1;

	if (freq >= 2412 && freq <= 2472) {
		if (chandef->width > NL80211_CHAN_WIDTH_40)
			return false;

		/* 2.407 GHz, channels 1..13 */
		if (chandef->width == NL80211_CHAN_WIDTH_40) {
			if (freq > chandef->chan->center_freq)
				*op_class = 83; /* HT40+ */
			else
				*op_class = 84; /* HT40- */
		} else {
			*op_class = 81;
		}

		return true;
	}

	if (freq == 2484) {
		if (chandef->width > NL80211_CHAN_WIDTH_40)
			return false;

		*op_class = 82; /* channel 14 */
		return true;
	}

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_80:
		vht_opclass = 128;
		break;
	case NL80211_CHAN_WIDTH_160:
		vht_opclass = 129;
		break;
	case NL80211_CHAN_WIDTH_80P80:
		vht_opclass = 130;
		break;
	case NL80211_CHAN_WIDTH_10:
	case NL80211_CHAN_WIDTH_5:
		return false; /* unsupported for now */
	default:
		vht_opclass = 0;
		break;
	}

	/* 5 GHz, channels 36..48 */
	if (freq >= 5180 && freq <= 5240) {
		if (vht_opclass) {
			*op_class = vht_opclass;
		} else if (chandef->width == NL80211_CHAN_WIDTH_40) {
			if (freq > chandef->chan->center_freq)
				*op_class = 116;
			else
				*op_class = 117;
		} else {
			*op_class = 115;
		}

		return true;
	}

	/* 5 GHz, channels 52..64 */
	if (freq >= 5260 && freq <= 5320) {
		if (vht_opclass) {
			*op_class = vht_opclass;
		} else if (chandef->width == NL80211_CHAN_WIDTH_40) {
			if (freq > chandef->chan->center_freq)
				*op_class = 119;
			else
				*op_class = 120;
		} else {
			*op_class = 118;
		}

		return true;
	}

	/* 5 GHz, channels 100..144 */
	if (freq >= 5500 && freq <= 5720) {
		if (vht_opclass) {
			*op_class = vht_opclass;
		} else if (chandef->width == NL80211_CHAN_WIDTH_40) {
			if (freq > chandef->chan->center_freq)
				*op_class = 122;
			else
				*op_class = 123;
		} else {
			*op_class = 121;
		}

		return true;
	}

	/* 5 GHz, channels 149..169 */
	if (freq >= 5745 && freq <= 5845) {
		if (vht_opclass) {
			*op_class = vht_opclass;
		} else if (chandef->width == NL80211_CHAN_WIDTH_40) {
			if (freq > chandef->chan->center_freq)
				*op_class = 126;
			else
				*op_class = 127;
		} else if (freq <= 5805) {
			*op_class = 124;
		} else {
			*op_class = 125;
		}

		return true;
	}

	/* 56.16 GHz, channel 1..4 */
	if (freq >= 56160 + 2160 * 1 && freq <= 56160 + 2160 * 4) {
		if (chandef->width >= NL80211_CHAN_WIDTH_40)
			return false;

		*op_class = 180;
		return true;
	}

	/* not supported yet */
	return false;
}

#ifndef U16_MAX
#define U16_MAX         ((u16)~0U)
#endif
#endif
