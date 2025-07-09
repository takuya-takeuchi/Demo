#include <iostream>
#include <vector>
#include <cstring>

#include <EthLayer.h>
#include <IPv4Layer.h>
#include <Packet.h>
#include <PayloadLayer.h>
#include <PcapFileDevice.h>
#include <PcapLiveDevice.h>
#include <PcapLiveDeviceList.h>
#include <UdpLayer.h>

int main(int argc, char* argv[])
{
    if (argc != 9)
    {
        std::cerr << "Usage: " << argv[0] << " pcapFile devName newSrcIp newDstIp srcMac dstMac srcPort dstPort" << std::endl;
        return 1;
    }

    std::string pcapFile = argv[1];
    std::string devName = argv[2];
    std::string newSrcIp = argv[3];
    std::string newDstIp = argv[4];
    pcpp::MacAddress srcMac(argv[5]);
    pcpp::MacAddress dstMac(argv[6]);
    uint16_t srcPort = static_cast<uint16_t>(std::stoi(argv[7]));
    uint16_t dstPort = static_cast<uint16_t>(std::stoi(argv[8]));
    
    std::cout << "pcapFile: " << pcapFile << std::endl;
    std::cout << " devName: " << devName << std::endl;
    std::cout << "  srcMac: " << newSrcIp << std::endl;
    std::cout << "  dstMac: " << newDstIp << std::endl;
    std::cout << "  srcMac: " << srcMac.toString() << std::endl;
    std::cout << "  dstMac: " << dstMac.toString() << std::endl;
    std::cout << " srcPort: " << srcPort << std::endl;
    std::cout << " dstPort: " << dstPort << std::endl;

    pcpp::PcapLiveDevice* dev = pcpp::PcapLiveDeviceList::getInstance().getPcapLiveDeviceByName(devName);
    if (!dev || !dev->open())
    {
        std::cerr << "Failed to open device: " << devName << std::endl;
        return 1;
    }

    pcpp::IFileReaderDevice* reader = pcpp::IFileReaderDevice::getReader(pcapFile);
    if (!reader->open())
    {
        std::cerr << "Cannot open file " << pcapFile << std::endl;
        dev->close();
        return 1;
    }

    size_t udp_count = 0;

    pcpp::RawPacket rawPacket;
    while (reader->getNextPacket(rawPacket))
    {
        pcpp::Packet parsedPacket(&rawPacket);

        auto udpLayer = parsedPacket.getLayerOfType<pcpp::UdpLayer>();
        auto ipLayer = parsedPacket.getLayerOfType<pcpp::IPv4Layer>();
        if (udpLayer && ipLayer)
        {
            pcpp::PayloadLayer* payloadLayer = parsedPacket.getLayerOfType<pcpp::PayloadLayer>();
            const uint8_t* payloadData = nullptr;
            size_t payloadLen = 0;
            if (payloadLayer != nullptr)
            {
                payloadData = payloadLayer->getPayload();
                payloadLen = payloadLayer->getPayloadLen();
            }

            pcpp::UdpLayer newUdpLayer(srcPort, dstPort);
            pcpp::IPv4Layer newIpLayer(pcpp::IPv4Address(newSrcIp.c_str()), pcpp::IPv4Address(newDstIp.c_str()));
            newIpLayer.getIPv4Header()->timeToLive = 64;

            pcpp::PayloadLayer newPayloadLayer(payloadData, payloadLen);
            pcpp::EthLayer ethLayer(srcMac, dstMac, PCPP_ETHERTYPE_IP);

            // The combined headers of the UDP, IP, and Ethernet layers are only a few dozen bytes in total,
            // so allocating around 100 bytes is generally sufficient (based on practical experience).
            pcpp::Packet newPacket(100 + payloadLen);
            newPacket.addLayer(&ethLayer);
            newPacket.addLayer(&newIpLayer);
            newPacket.addLayer(&newUdpLayer);
            newPacket.addLayer(&newPayloadLayer);

            newUdpLayer.computeCalculateFields();
            newIpLayer.computeCalculateFields();

            if (dev->sendPacket(&newPacket))
                ++udp_count;
        }
    }

    std::cout << "Sent " << udp_count << " UDP packets." << std::endl;
    
    dev->close();
    reader->close();
    delete reader;

    return 0;
}