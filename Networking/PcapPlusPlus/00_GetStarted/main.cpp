#include "Packet.h"
#include "PcapFileDevice.h"
#include "PcapLiveDevice.h"
#include "PcapLiveDeviceList.h"
#include "UdpLayer.h"
#include "IPv4Layer.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " input.pcapng new_src_ip interface" << std::endl;
        return 1;
    }

    const char* pcapFile = argv[1];
    std::string newSrcIp = argv[2];
    std::string iface = argv[3];

    // 開く
    pcpp::IFileReaderDevice* reader = pcpp::IFileReaderDevice::getReader(pcapFile);
    if (!reader->open()) {
        std::cerr << "Cannot open file " << pcapFile << std::endl;
        return 1;
    }

    // ライブデバイス取得
    pcpp::PcapLiveDevice* dev = pcpp::PcapLiveDeviceList::getInstance().getPcapLiveDeviceByName(iface);
    if (!dev) {
        std::cerr << "Cannot find interface " << iface << std::endl;
        return 1;
    }
    if (!dev->open()) {
        std::cerr << "Cannot open interface " << iface << std::endl;
        return 1;
    }

    pcpp::RawPacket rawPacket;
    size_t udp_count = 0;

    while (reader->getNextPacket(rawPacket)) 
    {
        pcpp::Packet parsedPacket(&rawPacket);

        auto udpLayer = parsedPacket.getLayerOfType<pcpp::UdpLayer>();
        auto ipLayer = parsedPacket.getLayerOfType<pcpp::IPv4Layer>();

        if (udpLayer && ipLayer) {
            // 送信元IPを書き換え
            ipLayer->setSrcIPv4Address(newSrcIp);

            // UDP/TCP/IPチェックサム再計算
            ipLayer->computeCalculateFields();

            // 送信
            dev->sendPacket(&parsedPacket);
            ++udp_count;
        }
    }

    std::cout << "Sent " << udp_count << " UDP packets." << std::endl;
    dev->close();
    reader->close();
    delete reader;
    return 0;
}