using System;
using System.Runtime.Remoting.Channels;
using Ch.Elca.Iiop;
using Ch.Elca.Iiop.Services;
using omg.org.CosNaming;

namespace hpp
{
    namespace corbaserver
    {
        namespace manipulation
        {
            public class Client
            {
                public Client()
                {
                }
        
                public void connect(string nameServiceHost)
                {
                    connect(nameServiceHost, 2809);
                }
        
                public void connect(string nameServiceHost, int nameServicePort)
                {
                    connect(nameServiceHost, nameServicePort, "");
                }
        
                //public void connect(string nameServiceHost, int nameServicePort = 2809, string context = "")
                public void connect(string nameServiceHost, int nameServicePort, string context)
                {
                    channel = new IiopClientChannel();
                    ChannelServices.RegisterChannel(channel, false);
        
                    // access OmniORB Name Service
                    CorbaInit init = CorbaInit.GetInit();
                    NamingContext nameService = init.GetNameService(nameServiceHost, nameServicePort);
        
                    NameComponent hppName = new NameComponent("hpp" + context, "corbaserver");
                    robot   = (Robot  )nameService.resolve(new NameComponent[] { hppName, new NameComponent("manipulation", "robot"  ) });
                    problem = (Problem)nameService.resolve(new NameComponent[] { hppName, new NameComponent("manipulation", "problem") });
                    graph   = (Graph  )nameService.resolve(new NameComponent[] { hppName, new NameComponent("manipulation", "graph"  ) });
                }
        
                private IiopClientChannel channel;
        
                public Robot   robot;
                public Problem problem;
                public Graph   graph;
            };

        }
    }
}
