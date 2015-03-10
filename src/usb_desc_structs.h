/* 
 * File:   usb_desc_structs.h
 * Author: fox
 *
 * Created on 23. únor 2015, 13:31
 */

#ifndef USB_DESC_STRUCTS_H
#define	USB_DESC_STRUCTS_H


#ifdef	__cplusplus
extern "C" {
#endif

    enum {
        USB_DSC_DEVICE = 0x01,
        USB_DSC_CONFIGURATION = 0x02,
        USB_DSC_STRING = 0x03,
        USB_DSC_INTERFACE = 0x04,
        USB_DSC_ENDPOINT = 0x05,
    }USB_descriptor_types;

    typedef struct
    {
        uint8_t     bLength;            //Size of this descriptor in bytes
        uint8_t     bDescriptorType;    //DEVICE Descriptor Type
        uint16_t    bcdUSB;             //USB Specification Release Number in Binary-Coded Decimal (i.e., 2.10 is 210H).
        uint8_t     bDeviceClass;       //If this field is set to FFH, the device class is vendor-specific.
        uint8_t     bDeviceSubClass;    //
        uint8_t     bDeviceProtocol;    //If this field is set to FFH, the device uses a vendor-specific protocol on a device basis.
        uint8_t     bMaxPacketSize0;    //Maximum packet size for endpoint zero(only 8, 16, 32, or 64 are valid)
        uint16_t    idVendor;
        uint16_t    idProduct;
        uint16_t    bcdDevice;          //Device release number in binary-coded decimal
        uint8_t     iManufacturer;      //string index
        uint8_t     iProduct;           //string index
        uint8_t     iSerialNumber;      //string index
        uint8_t     bNumConfigurations; //Number of possible configurations

    }USB_dev_decriptor;  //usb spec p262

    typedef struct
    {
        uint8_t     bLength;            //Size of this descriptor in bytes
        uint8_t     bDescriptorType;    //CONFIGURATION descriptor type (= 2)
        uint16_t    wTotalLength;       //Total number of bytes in this descriptor and all the following descriptors.
        uint8_t     bNumInterfaces;     //Number of interfaces supported by this configuration
        uint8_t     bConfigurationValue;//Value used by Set Configuration to select this configuration
        uint8_t     iConfiguration;     //Index of string descriptor describing configuration
        uint8_t     bmAttributes;       /*D7: Must be set to 1
                                          D6: Self-powered
                                          D5: Remote Wakeup
                                          D4...D0: Set to 0*/
        uint8_t     bMaxPower;          //Maximum current drawn by device in this configuration. In units of 2mA. So 50 means 100 mA.
    }USB_configuration_decriptor;  //usb spec p?? http://www.usbmadesimple.co.uk/ums_4.htm

    typedef struct
    {

    }USB_interface_decriptor;

    typedef struct
    {
        uint8_t     bLength;            //Size of this descriptor in bytes
        uint8_t     bDescriptorType;    //CONFIGURATION descriptor type (= 3)
        //UNICODE encoded string here
    }USB_string_descriptor_hdr;


#ifdef	__cplusplus
}
#endif


#endif	/* USB_DESC_STRUCTS_H */

