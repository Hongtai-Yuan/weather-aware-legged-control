#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x2c635209, "module_layout" },
	{ 0x76537aa2, "can_change_mtu" },
	{ 0xe180ce57, "usb_deregister" },
	{ 0x942410fb, "usb_register_driver" },
	{ 0x8a73ef4d, "alloc_can_skb" },
	{ 0xf12d9387, "can_fd_dlc2len" },
	{ 0x49d1e765, "alloc_canfd_skb" },
	{ 0x105b7cb4, "can_rx_offload_irq_finish" },
	{ 0x48330eee, "alloc_can_err_skb" },
	{ 0x6a2bfc93, "netif_tx_wake_queue" },
	{ 0xde0a8933, "can_rx_offload_get_echo_skb" },
	{ 0x6f9e763b, "timecounter_read" },
	{ 0x32cae849, "can_rx_offload_queue_tail" },
	{ 0x2b426bb9, "can_rx_offload_queue_sorted" },
	{ 0xbc3f2cb0, "timecounter_cyc2time" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x862258db, "timecounter_init" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x20e708d2, "close_candev" },
	{ 0xabdba6e3, "napi_disable" },
	{ 0x6e45cc7d, "can_rx_offload_enable" },
	{ 0x90943690, "open_candev" },
	{ 0x962c8ae1, "usb_kill_anchored_urbs" },
	{ 0x8f548ec0, "unregister_candev" },
	{ 0x93c7edeb, "usb_find_common_endpoints" },
	{ 0x37a0cba, "kfree" },
	{ 0xc4cffb2c, "netdev_err" },
	{ 0x5726067d, "netif_device_detach" },
	{ 0x56baf789, "usb_unanchor_urb" },
	{ 0xf1fbcc18, "can_free_echo_skb" },
	{ 0xa487e741, "consume_skb" },
	{ 0x6047ede6, "can_fd_len2dlc" },
	{ 0x2f886acb, "usb_free_urb" },
	{ 0xb20c3797, "usb_submit_urb" },
	{ 0x6bf8dc58, "can_put_echo_skb" },
	{ 0x2cf05418, "usb_anchor_urb" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x437b7175, "usb_alloc_urb" },
	{ 0x10bc6bea, "kfree_skb_reason" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x805b0f1a, "free_candev" },
	{ 0xa3d346c7, "can_rx_offload_del" },
	{ 0x86219045, "register_candev" },
	{ 0xae0e712b, "can_rx_offload_add_manual" },
	{ 0xe2d5255a, "strcmp" },
	{ 0xec2dcbc3, "_dev_info" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xc2d2fffc, "alloc_candev_mqs" },
	{ 0x242c6648, "netdev_info" },
	{ 0xe22b65d5, "ethtool_op_get_ts_info" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xf0311d7, "_dev_err" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0x991f59c4, "usb_control_msg_recv" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x1d7dd30, "usb_control_msg_send" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("usb:v1D50p606Fd*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v1209p2323d*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v1CD2p606Fd*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v16D0p10B8d*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v16D0p0F30d*dc*dsc*dp*ic*isc*ip*in00*");

MODULE_INFO(srcversion, "CE3815164FE99561129E993");
