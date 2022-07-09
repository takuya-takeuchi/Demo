# SOAP

## Abstacts

* How to use SOAP interface on ASP.NET
* Change default namespace `http://tempuri.org/` to `http://demo.com/helloservice/`

## Requirements

* Visual Studio 2022
* .NET 6.0 SDK

## Dependencies

* [SoapCore](https://github.com/DigDes/SoapCore)
  * MIT License

## How to use?

````powershell
$ pwsh .\Run.ps1
ビルドしています...
info: Microsoft.Hosting.Lifetime[14]
      Now listening on: https://localhost:5001
info: Microsoft.Hosting.Lifetime[14]
      Now listening on: http://localhost:5000
info: Microsoft.Hosting.Lifetime[0]
      Application started. Press Ctrl+C to shut down.
info: Microsoft.Hosting.Lifetime[0]
      Hosting environment: Development
info: Microsoft.Hosting.Lifetime[0]
      Content root path: D:\Works\OpenSource\Demo\ASP.NET\02_SOAP\sources\Demo
````

And open `https://localhost:5001/HelloService.asmx` on browser.

## Result

````xml
<wsdl:definitions xmlns:soap="http://schemas.xmlsoap.org/wsdl/soap/"
    xmlns:tns="http://demo.com/helloservice/"
    xmlns:xsd="http://www.w3.org/2001/XMLSchema"
    xmlns:http="http://schemas.microsoft.com/ws/06/2004/policy/http"
    xmlns:msc="http://schemas.microsoft.com/ws/2005/12/wsdl/contract"
    xmlns:wsp="http://schemas.xmlsoap.org/ws/2004/09/policy"
    xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd"
    xmlns:wsam="http://www.w3.org/2007/05/addressing/metadata"
    xmlns:wsdl="http://schemas.xmlsoap.org/wsdl/" targetNamespace="http://demo.com/helloservice/" name="IHelloService">
    <wsdl:types>
        <xsd:schema elementFormDefault="qualified" targetNamespace="http://demo.com/helloservice/">
            <xsd:import namespace="http://schemas.microsoft.com/2003/10/Serialization/Arrays"/>
            <xsd:import namespace="http://schemas.datacontract.org/2004/07/System"/>
            <xsd:element name="Hello">
                <xsd:complexType>
                    <xsd:sequence>
                        <xsd:element minOccurs="0" maxOccurs="1" name="menamessage" type="xsd:string"/>
                    </xsd:sequence>
                </xsd:complexType>
            </xsd:element>
            <xsd:element name="HelloResponse">
                <xsd:complexType>
                    <xsd:sequence>
                        <xsd:element minOccurs="0" maxOccurs="1" name="HelloResult" type="xsd:string"/>
                    </xsd:sequence>
                </xsd:complexType>
            </xsd:element>
        </xsd:schema>
    </wsdl:types>
    <wsdl:message name="IHelloService_Hello_InputMessage">
        <wsdl:part name="parameters" element="tns:Hello"/>
    </wsdl:message>
    <wsdl:message name="IHelloService_Hello_OutputMessage">
        <wsdl:part name="parameters" element="tns:HelloResponse"/>
    </wsdl:message>
    <wsdl:portType name="IHelloService">
        <wsdl:operation name="Hello">
            <wsdl:input message="tns:IHelloService_Hello_InputMessage"/>
            <wsdl:output message="tns:IHelloService_Hello_OutputMessage"/>
        </wsdl:operation>
    </wsdl:portType>
    <wsdl:binding name="BasicHttpBinding_IHelloService_soap" type="tns:IHelloService">
        <soap:binding transport="http://schemas.xmlsoap.org/soap/http"/>
        <wsdl:operation name="Hello">
            <soap:operation soapAction="http://demo.com/helloservice/IHelloService/Hello" style="document"/>
            <wsdl:input>
                <soap:body use="literal"/>
            </wsdl:input>
            <wsdl:output>
                <soap:body use="literal"/>
            </wsdl:output>
        </wsdl:operation>
    </wsdl:binding>
    <wsdl:service name="IHelloService">
        <wsdl:port name="BasicHttpBinding_IHelloService_soap" binding="tns:BasicHttpBinding_IHelloService_soap">
            <soap:address location="https://localhost:44345/HelloService.asmx"/>
        </wsdl:port>
    </wsdl:service>
</wsdl:definitions>
````