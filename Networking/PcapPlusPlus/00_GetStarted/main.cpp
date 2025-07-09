#include <iostream>

#include <PcapLiveDeviceList.h>

int main()
{
    std::vector<pcpp::PcapLiveDevice*> devList = pcpp::PcapLiveDeviceList::getInstance().getPcapLiveDevicesList();
    for (auto dev : devList)
        std::cout << dev->getName() << " : " << dev->getDesc() << std::endl;
        
    return 0;
}