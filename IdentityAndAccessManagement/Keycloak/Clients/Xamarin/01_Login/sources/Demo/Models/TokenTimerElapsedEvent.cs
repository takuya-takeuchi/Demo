using System;
using Prism.Events;

namespace Demo.Models
{

    public sealed class TokenTimerElapsedEvent : PubSubEvent<DateTimeOffset?>
    {
    }

}