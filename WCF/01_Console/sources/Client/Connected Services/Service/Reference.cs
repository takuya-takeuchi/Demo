﻿//------------------------------------------------------------------------------
// <auto-generated>
//     このコードはツールによって生成されました。
//     ランタイム バージョン:4.0.30319.42000
//
//     このファイルへの変更は、以下の状況下で不正な動作の原因になったり、
//     コードが再生成されるときに損失したりします。
// </auto-generated>
//------------------------------------------------------------------------------

namespace Client.Service {
    using System.Runtime.Serialization;
    using System;
    
    
    [System.Diagnostics.DebuggerStepThroughAttribute()]
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.Runtime.Serialization", "4.0.0.0")]
    [System.Runtime.Serialization.DataContractAttribute(Name="Greeting", Namespace="http://schemas.datacontract.org/2004/07/Server.Models")]
    [System.SerializableAttribute()]
    public partial class Greeting : object, System.Runtime.Serialization.IExtensibleDataObject, System.ComponentModel.INotifyPropertyChanged {
        
        [System.NonSerializedAttribute()]
        private System.Runtime.Serialization.ExtensionDataObject extensionDataField;
        
        [System.Runtime.Serialization.OptionalFieldAttribute()]
        private string MessageField;
        
        [global::System.ComponentModel.BrowsableAttribute(false)]
        public System.Runtime.Serialization.ExtensionDataObject ExtensionData {
            get {
                return this.extensionDataField;
            }
            set {
                this.extensionDataField = value;
            }
        }
        
        [System.Runtime.Serialization.DataMemberAttribute()]
        public string Message {
            get {
                return this.MessageField;
            }
            set {
                if ((object.ReferenceEquals(this.MessageField, value) != true)) {
                    this.MessageField = value;
                    this.RaisePropertyChanged("Message");
                }
            }
        }
        
        public event System.ComponentModel.PropertyChangedEventHandler PropertyChanged;
        
        protected void RaisePropertyChanged(string propertyName) {
            System.ComponentModel.PropertyChangedEventHandler propertyChanged = this.PropertyChanged;
            if ((propertyChanged != null)) {
                propertyChanged(this, new System.ComponentModel.PropertyChangedEventArgs(propertyName));
            }
        }
    }
    
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "4.0.0.0")]
    [System.ServiceModel.ServiceContractAttribute(ConfigurationName="Service.IGreetingService")]
    public interface IGreetingService {
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IGreetingService/Greet", ReplyAction="http://tempuri.org/IGreetingService/GreetResponse")]
        Client.Service.Greeting Greet(Client.Service.Greeting greeting);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IGreetingService/Greet", ReplyAction="http://tempuri.org/IGreetingService/GreetResponse")]
        System.Threading.Tasks.Task<Client.Service.Greeting> GreetAsync(Client.Service.Greeting greeting);
    }
    
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "4.0.0.0")]
    public interface IGreetingServiceChannel : Client.Service.IGreetingService, System.ServiceModel.IClientChannel {
    }
    
    [System.Diagnostics.DebuggerStepThroughAttribute()]
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "4.0.0.0")]
    public partial class GreetingServiceClient : System.ServiceModel.ClientBase<Client.Service.IGreetingService>, Client.Service.IGreetingService {
        
        public GreetingServiceClient() {
        }
        
        public GreetingServiceClient(string endpointConfigurationName) : 
                base(endpointConfigurationName) {
        }
        
        public GreetingServiceClient(string endpointConfigurationName, string remoteAddress) : 
                base(endpointConfigurationName, remoteAddress) {
        }
        
        public GreetingServiceClient(string endpointConfigurationName, System.ServiceModel.EndpointAddress remoteAddress) : 
                base(endpointConfigurationName, remoteAddress) {
        }
        
        public GreetingServiceClient(System.ServiceModel.Channels.Binding binding, System.ServiceModel.EndpointAddress remoteAddress) : 
                base(binding, remoteAddress) {
        }
        
        public Client.Service.Greeting Greet(Client.Service.Greeting greeting) {
            return base.Channel.Greet(greeting);
        }
        
        public System.Threading.Tasks.Task<Client.Service.Greeting> GreetAsync(Client.Service.Greeting greeting) {
            return base.Channel.GreetAsync(greeting);
        }
    }
}
