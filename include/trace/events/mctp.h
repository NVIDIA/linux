/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mctp

#if !defined(_TRACE_MCTP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MCTP_H

#include <linux/tracepoint.h>

#ifndef __TRACE_MCTP_ENUMS
#define __TRACE_MCTP_ENUMS
enum {
	MCTP_TRACE_KEY_TIMEOUT,
	MCTP_TRACE_KEY_REPLIED,
	MCTP_TRACE_KEY_INVALIDATED,
	MCTP_TRACE_KEY_CLOSED,
	MCTP_TRACE_KEY_DROPPED,
	MCTP_TRACE_KEY_TX_PARTIAL,
};
#endif /* __TRACE_MCTP_ENUMS */

TRACE_DEFINE_ENUM(MCTP_TRACE_KEY_TIMEOUT);
TRACE_DEFINE_ENUM(MCTP_TRACE_KEY_REPLIED);
TRACE_DEFINE_ENUM(MCTP_TRACE_KEY_INVALIDATED);
TRACE_DEFINE_ENUM(MCTP_TRACE_KEY_CLOSED);
TRACE_DEFINE_ENUM(MCTP_TRACE_KEY_DROPPED);
TRACE_DEFINE_ENUM(MCTP_TRACE_KEY_TX_PARTIAL);

TRACE_EVENT(mctp_key_acquire,
	TP_PROTO(const struct mctp_sk_key *key),
	TP_ARGS(key),
	TP_STRUCT__entry(
		__field(__u8,	paddr)
		__field(__u8,	laddr)
		__field(__u8,	tag)
	),
	TP_fast_assign(
		__entry->paddr = key->peer_addr;
		__entry->laddr = key->local_addr;
		__entry->tag = key->tag;
	),
	TP_printk("local %d, peer %d, tag %1x",
		__entry->laddr,
		__entry->paddr,
		__entry->tag
	)
);

TRACE_EVENT(mctp_key_release,
	TP_PROTO(const struct mctp_sk_key *key, int reason),
	TP_ARGS(key, reason),
	TP_STRUCT__entry(
		__field(__u8,	paddr)
		__field(__u8,	laddr)
		__field(__u8,	tag)
		__field(int,	reason)
	),
	TP_fast_assign(
		__entry->paddr = key->peer_addr;
		__entry->laddr = key->local_addr;
		__entry->tag = key->tag;
		__entry->reason = reason;
	),
	TP_printk("local %d, peer %d, tag %1x %s",
		__entry->laddr,
		__entry->paddr,
		__entry->tag,
		__print_symbolic(__entry->reason,
				 { MCTP_TRACE_KEY_TIMEOUT, "timeout" },
				 { MCTP_TRACE_KEY_REPLIED, "replied" },
				 { MCTP_TRACE_KEY_INVALIDATED, "invalidated" },
				 { MCTP_TRACE_KEY_CLOSED, "closed" },
				 { MCTP_TRACE_KEY_DROPPED, "dropped" },
				 { MCTP_TRACE_KEY_TX_PARTIAL, "tx-partial" })
	)
);

TRACE_EVENT(mctp_rx_packet,
	TP_PROTO(struct sk_buff *skb),
	TP_ARGS(skb),
	TP_STRUCT__entry(
		__field(u32, len)
		__field(u8, src)
		__field(u8, dest)
		__field(u8, flags_seq_tag)
		__array(u8, data, 32)
	),
	TP_fast_assign(
		struct mctp_hdr *mh = skb->len >= sizeof(*mh) ?
				      mctp_hdr(skb) : NULL;
		__entry->len = skb->len;
		__entry->src = mh ? mh->src : 0;
		__entry->dest = mh ? mh->dest : 0;
		__entry->flags_seq_tag = mh ? mh->flags_seq_tag : 0;
		memcpy(__entry->data, skb->data, min_t(u32, skb->len, 32));
	),
	TP_printk("RX: src=%02x dst=%02x flags=%02x len=%u data=%s",
		__entry->src, __entry->dest, __entry->flags_seq_tag,
		__entry->len,
		__print_hex(__entry->data, min_t(u32, __entry->len, 32))
	)
);

TRACE_EVENT(mctp_rx_socket,
	TP_PROTO(struct sk_buff *skb, int rc),
	TP_ARGS(skb, rc),
	TP_STRUCT__entry(
		__field(u32, len)
		__field(int, rc)
		__array(u8, data, 32)
	),
	TP_fast_assign(
		__entry->len = skb->len;
		__entry->rc = rc;
		memcpy(__entry->data, skb->data, min_t(u32, skb->len, 32));
	),
	TP_printk("RX_SOCKET: len=%u rc=%d data=%s",
		__entry->len, __entry->rc,
		__print_hex(__entry->data, min_t(u32, __entry->len, 32))
	)
);

TRACE_EVENT(mctp_tx_packet,
	TP_PROTO(struct sk_buff *skb),
	TP_ARGS(skb),
	TP_STRUCT__entry(
		__field(u32, len)
		__array(u8, data, 32)
	),
	TP_fast_assign(
		__entry->len = skb->len;
		memcpy(__entry->data, skb->data, min_t(u32, skb->len, 32));
	),
	TP_printk("TX: len=%u data=%s",
		__entry->len,
		__print_hex(__entry->data, min_t(u32, __entry->len, 32))
	)
);

TRACE_EVENT(mctp_drop_packet,
	TP_PROTO(struct sk_buff *skb, const char *reason),
	TP_ARGS(skb, reason),
	TP_STRUCT__entry(
		__field(u32, len)
		__field(u8, src)
		__field(u8, dest)
		__string(reason, reason)
		__array(u8, data, 16)
	),
	TP_fast_assign(
		struct mctp_hdr *mh = skb->len >= sizeof(*mh) ? mctp_hdr(skb) : NULL;
		__entry->len = skb->len;
		__entry->src = mh ? mh->src : 0;
		__entry->dest = mh ? mh->dest : 0;
		__assign_str(reason);
		memcpy(__entry->data, skb->data, min_t(u32, skb->len, 16));
	),
	TP_printk("DROP at %s: src=%02x dst=%02x len=%u data=%s",
		__get_str(reason), __entry->src, __entry->dest, __entry->len,
		__print_hex(__entry->data, min_t(u32, __entry->len, 16))
	)
);

TRACE_EVENT(mctp_device_register,
	TP_PROTO(const struct net_device *dev, unsigned int net_id),
	TP_ARGS(dev, net_id),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(unsigned int, net_id)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->net_id = net_id;
	),
	TP_printk("dev=%s ifindex=%d net=%u",
		__get_str(name), __entry->ifindex, __entry->net_id
	)
);

TRACE_EVENT(mctp_device_unregister,
	TP_PROTO(const struct net_device *dev),
	TP_ARGS(dev),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
	),
	TP_printk("dev=%s ifindex=%d",
		__get_str(name), __entry->ifindex
	)
);

TRACE_EVENT(mctp_address_add,
	TP_PROTO(const struct net_device *dev, u8 addr),
	TP_ARGS(dev, addr),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, addr)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->addr = addr;
	),
	TP_printk("dev=%s ifindex=%d addr=%02x",
		__get_str(name), __entry->ifindex, __entry->addr
	)
);

TRACE_EVENT(mctp_address_del,
	TP_PROTO(const struct net_device *dev, u8 addr),
	TP_ARGS(dev, addr),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, addr)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->addr = addr;
	),
	TP_printk("dev=%s ifindex=%d addr=%02x",
		__get_str(name), __entry->ifindex, __entry->addr
	)
);

TRACE_EVENT(mctp_neighbor_add,
	TP_PROTO(const struct net_device *dev, u8 eid, const void *lladdr, size_t len),
	TP_ARGS(dev, eid, lladdr, len),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, eid)
		__field(size_t, lladdr_len)
		__array(u8, lladdr, MAX_ADDR_LEN)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->eid = eid;
		__entry->lladdr_len = len;
		memcpy(__entry->lladdr, lladdr, min_t(size_t, len, MAX_ADDR_LEN));
	),
	TP_printk("dev=%s ifindex=%d eid=%02x lladdr=%s",
		__get_str(name), __entry->ifindex, __entry->eid,
		__print_hex(__entry->lladdr, min_t(size_t, __entry->lladdr_len, MAX_ADDR_LEN))
	)
);

TRACE_EVENT(mctp_neighbor_del,
	TP_PROTO(const struct net_device *dev, u8 eid),
	TP_ARGS(dev, eid),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, eid)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->eid = eid;
	),
	TP_printk("dev=%s ifindex=%d eid=%02x",
		__get_str(name), __entry->ifindex, __entry->eid
	)
);

TRACE_EVENT(mctp_neighbor_lookup,
	TP_PROTO(const struct net_device *dev, u8 eid, int rc),
	TP_ARGS(dev, eid, rc),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, eid)
		__field(int, rc)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->eid = eid;
		__entry->rc = rc;
	),
	TP_printk("dev=%s ifindex=%d eid=%02x rc=%d",
		__get_str(name), __entry->ifindex, __entry->eid, __entry->rc
	)
);

TRACE_EVENT(mctp_route_add,
	TP_PROTO(const struct net_device *dev, u8 daddr_start, u8 daddr_extent,
		 unsigned int mtu),
	TP_ARGS(dev, daddr_start, daddr_extent, mtu),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, daddr_start)
		__field(u8, daddr_extent)
		__field(unsigned int, mtu)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->daddr_start = daddr_start;
		__entry->daddr_extent = daddr_extent;
		__entry->mtu = mtu;
	),
	TP_printk("dev=%s ifindex=%d daddr=%02x-%02x mtu=%u",
		__get_str(name), __entry->ifindex, __entry->daddr_start,
		__entry->daddr_start + __entry->daddr_extent, __entry->mtu
	)
);

TRACE_EVENT(mctp_route_del,
	TP_PROTO(const struct net_device *dev, u8 daddr_start, u8 daddr_extent),
	TP_ARGS(dev, daddr_start, daddr_extent),
	TP_STRUCT__entry(
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, daddr_start)
		__field(u8, daddr_extent)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->daddr_start = daddr_start;
		__entry->daddr_extent = daddr_extent;
	),
	TP_printk("dev=%s ifindex=%d daddr=%02x-%02x",
		__get_str(name), __entry->ifindex, __entry->daddr_start,
		__entry->daddr_start + __entry->daddr_extent
	)
);

TRACE_EVENT(mctp_route_lookup,
	TP_PROTO(u8 daddr, const struct net_device *dev, bool found),
	TP_ARGS(daddr, dev, found),
	TP_STRUCT__entry(
		__field(u8, daddr)
		__string(name, dev ? dev->name : "none")
		__field(int, ifindex)
		__field(bool, found)
	),
	TP_fast_assign(
		__entry->daddr = daddr;
		__assign_str(name);
		__entry->ifindex = dev ? dev->ifindex : 0;
		__entry->found = found;
	),
	TP_printk("daddr=%02x dev=%s ifindex=%d found=%d",
		__entry->daddr, __get_str(name), __entry->ifindex, __entry->found
	)
);

TRACE_EVENT(mctp_route_output,
	TP_PROTO(struct sk_buff *skb, const struct net_device *dev),
	TP_ARGS(skb, dev),
	TP_STRUCT__entry(
		__field(u32, len)
		__field(u8, src)
		__field(u8, dest)
		__string(name, dev->name)
		__field(int, ifindex)
	),
	TP_fast_assign(
		struct mctp_hdr *mh = skb->len >= sizeof(*mh) ? mctp_hdr(skb) : NULL;
		__entry->len = skb->len;
		__entry->src = mh ? mh->src : 0;
		__entry->dest = mh ? mh->dest : 0;
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
	),
	TP_printk("dev=%s ifindex=%d src=%02x dst=%02x len=%u",
		__get_str(name), __entry->ifindex, __entry->src, __entry->dest, __entry->len
	)
);

TRACE_EVENT(mctp_local_output,
	TP_PROTO(u8 src, u8 dest, u8 tag, size_t len),
	TP_ARGS(src, dest, tag, len),
	TP_STRUCT__entry(
		__field(u8, src)
		__field(u8, dest)
		__field(u8, tag)
		__field(size_t, len)
	),
	TP_fast_assign(
		__entry->src = src;
		__entry->dest = dest;
		__entry->tag = tag;
		__entry->len = len;
	),
	TP_printk("src=%02x dst=%02x tag=%02x len=%zu",
		__entry->src, __entry->dest, __entry->tag, __entry->len
	)
);

TRACE_EVENT(mctp_fragment,
	TP_PROTO(u8 src, u8 dest, u8 seq, size_t frag_len, size_t total_len),
	TP_ARGS(src, dest, seq, frag_len, total_len),
	TP_STRUCT__entry(
		__field(u8, src)
		__field(u8, dest)
		__field(u8, seq)
		__field(size_t, frag_len)
		__field(size_t, total_len)
	),
	TP_fast_assign(
		__entry->src = src;
		__entry->dest = dest;
		__entry->seq = seq;
		__entry->frag_len = frag_len;
		__entry->total_len = total_len;
	),
	TP_printk("src=%02x dst=%02x seq=%u frag_len=%zu total_len=%zu",
		__entry->src, __entry->dest, __entry->seq, __entry->frag_len, __entry->total_len
	)
);

TRACE_EVENT(mctp_reassemble_start,
	TP_PROTO(u8 src, u8 dest, u8 seq),
	TP_ARGS(src, dest, seq),
	TP_STRUCT__entry(
		__field(u8, src)
		__field(u8, dest)
		__field(u8, seq)
	),
	TP_fast_assign(
		__entry->src = src;
		__entry->dest = dest;
		__entry->seq = seq;
	),
	TP_printk("src=%02x dst=%02x seq=%u",
		__entry->src, __entry->dest, __entry->seq
	)
);

TRACE_EVENT(mctp_reassemble_finish,
	TP_PROTO(u8 src, u8 dest, size_t total_len),
	TP_ARGS(src, dest, total_len),
	TP_STRUCT__entry(
		__field(u8, src)
		__field(u8, dest)
		__field(size_t, total_len)
	),
	TP_fast_assign(
		__entry->src = src;
		__entry->dest = dest;
		__entry->total_len = total_len;
	),
	TP_printk("src=%02x dst=%02x total_len=%zu",
		__entry->src, __entry->dest, __entry->total_len
	)
);

TRACE_EVENT(mctp_transport_tx,
	TP_PROTO(const char *transport, const struct net_device *dev,
		 u8 dest_addr, size_t len),
	TP_ARGS(transport, dev, dest_addr, len),
	TP_STRUCT__entry(
		__string(transport, transport)
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, dest_addr)
		__field(size_t, len)
	),
	TP_fast_assign(
		__assign_str(transport);
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->dest_addr = dest_addr;
		__entry->len = len;
	),
	TP_printk("%s dev=%s ifindex=%d dest=%02x len=%zu",
		__get_str(transport), __get_str(name), __entry->ifindex,
		__entry->dest_addr, __entry->len
	)
);

TRACE_EVENT(mctp_transport_rx,
	TP_PROTO(const char *transport, const struct net_device *dev,
		 u8 src_addr, size_t len),
	TP_ARGS(transport, dev, src_addr, len),
	TP_STRUCT__entry(
		__string(transport, transport)
		__string(name, dev->name)
		__field(int, ifindex)
		__field(u8, src_addr)
		__field(size_t, len)
	),
	TP_fast_assign(
		__assign_str(transport);
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__entry->src_addr = src_addr;
		__entry->len = len;
	),
	TP_printk("%s dev=%s ifindex=%d src=%02x len=%zu",
		__get_str(transport), __get_str(name), __entry->ifindex,
		__entry->src_addr, __entry->len
	)
);

TRACE_EVENT(mctp_transport_error,
	TP_PROTO(const char *transport, const struct net_device *dev,
		 const char *error, int rc),
	TP_ARGS(transport, dev, error, rc),
	TP_STRUCT__entry(
		__string(transport, transport)
		__string(name, dev->name)
		__field(int, ifindex)
		__string(error, error)
		__field(int, rc)
	),
	TP_fast_assign(
		__assign_str(transport);
		__assign_str(name);
		__entry->ifindex = dev->ifindex;
		__assign_str(error);
		__entry->rc = rc;
	),
	TP_printk("%s dev=%s ifindex=%d error=%s rc=%d",
		__get_str(transport), __get_str(name), __entry->ifindex,
		__get_str(error), __entry->rc
	)
);

#endif

#include <trace/define_trace.h>
