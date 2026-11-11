use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::State;
use speedybee_f405_v4_bsp as bsp;
use static_cell::StaticCell;

#[embassy_executor::task]
pub async fn run(usb: bsp::parts::UsbParts<'static>, irqs: super::Irqs) {
    static EP_OUT: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 128]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 16]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL: StaticCell<[u8; 64]> = StaticCell::new();
    static BUFFERS: StaticCell<bsp::usb::UsbCdcAcmBuffers<'static>> = StaticCell::new();
    static STATE: StaticCell<State<'static>> = StaticCell::new();

    let config = bsp::usb::UsbCdcAcmConfig {
        product: Some("SpeedyBee F405 V4 AHRS"),
        max_packet_size: embassy_usb_logger::MAX_PACKET_SIZE as u16,
        ..Default::default()
    };
    let buffers = BUFFERS.init(bsp::usb::UsbCdcAcmBuffers {
        ep_out: EP_OUT.init([0; 256]),
        config_descriptor: CONFIG_DESCRIPTOR.init([0; 128]),
        bos_descriptor: BOS_DESCRIPTOR.init([0; 16]),
        msos_descriptor: MSOS_DESCRIPTOR.init([0; 256]),
        control: CONTROL.init([0; 64]),
    });
    let bsp::usb::UsbCdcAcm { mut device, class } =
        usb.into_usb_cdc_acm(irqs, &config, buffers, STATE.init(State::new()));
    let logger = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class);
    join(device.run(), logger).await;
}
