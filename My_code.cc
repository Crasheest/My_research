/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/aodv-module.h"

#include "ns3/point-to-point-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/config-store-module.h"

#include "ns3/energy-module.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <iomanip>
/////


#define	PROG_DIR "out/3x3_random/"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MeshDot11sSim");

class MeshDot11sSim
{
    public:
	// Init test
	MeshDot11sSim ();

	// Run test
	void RunSim (int argc, char **argv);
	void Configure (int argc, char ** argv);

    //private:
	uint32_t m_xSize; //x size of the grid
	uint32_t m_ySize; //y size of the grid
	double   m_step;  //separation between nodes
	double   m_randomStart;
	double   m_totalTime;
	uint16_t m_packetSize;
	uint32_t m_nIfaces;
	bool     m_chan;
	std::string m_txrate;
	std::string m_root;
	//bool     m_showRtable;
        bool m_tracing;  
        int nodeSpeed;
        int nodePause;



	//to calculate the lenght of the simulation
	float m_timeTotal, m_timeStart, m_timeEnd;
	// List of network nodes
	NodeContainer meshNodes;
	//NodeContainer staNodes;
	Ptr<Node>     *n;
	Ptr<Node>     bgr;

	// List of all mesh point devices
	NetDeviceContainer meshDevices;
	NetDeviceContainer p2pDevices;

	// MeshHelper. Report is not static methods
	MeshHelper mesh;

    //private:
//энергетика
        double m_txPowerStart;
        double m_txPowerEnd;
        double m_txPowerLevels;
        double m_txPower;
        double initialEnergy;
        double voltage;
        double txCurrent;
        double rxCurrent;
        double idleCurrent;
        double sleepCurrent;
    //private:	
        EnergySourceContainer AttachEnergyModelToDevices();
	void NotifyDrained();
	double m_remainingEnergy;
	bool   m_energyDrained;
	uint32_t  m_traceNum;


	// Create nodes and setup their mobility
	void CreateTopologyNodes ();

	// Configure Mesh Network layer
	void ConfigureMeshLayer ();

	// Install internet stack on nodes
	void InstallInternetStack ();

	// Install applications randomly
	void SetUpUdpApplication();
	void SetUpTcpApplication();

	//void showHwmpRoutingTables(Ptr <Node>);
	//void showHwmpRoutingTables();

	// report throughput, delay, and pdf
	void FlowMonitoring();
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor;

	// Print mesh devices diagnostics
	void Report ();
	void RouteDiscoveryTimeSniffer (std::string context, Time time);
};


MeshDot11sSim::MeshDot11sSim () :
	m_xSize       (3),
	m_ySize       (3),
	m_step        (100),
	m_randomStart (0.1),
	m_totalTime   (65),
	m_packetSize  (512),//байт
	m_nIfaces     (1),
	m_chan        (true),
	m_txrate      ("64kbps"),//64000 бит/с
	m_root        ("ff:ff:ff:ff:ff:ff"),//клиент->сервер через корневой узел идет, обратно по наименьшим метрикам
	//m_showRtable  (true),
        m_tracing     (true),
        nodeSpeed     (2), //в м/с
        nodePause     (0), //в секундах

//энергетика
        m_txPowerStart (0.0),
        m_txPowerEnd  (16.0),
        m_txPowerLevels (17.0),
        m_txPower     (16.0), // в дБм
        initialEnergy (2500),// в милиАмпер*час
        voltage       (3.7),// в Вольтах
        txCurrent     (0.240),// в Амперах
        rxCurrent     (0.100),// в Амперах
        idleCurrent   (0.05),// в Амперах
        sleepCurrent  (0.002)// в Амперах
{
	NS_LOG_FUNCTION(this);

	m_energyDrained = false;// изменять нельзя! отвечает за сигнализацию отсутствия питания
	m_traceNum = 0;
}

void
MeshDot11sSim::Configure (int argc, char *argv[])
{
	NS_LOG_FUNCTION(this);

	CommandLine cmd;
	cmd.AddValue ("xSize"     , "number of nodes in each row" , m_xSize);
	cmd.AddValue ("ySize"     , "number of nodes in each column" , m_ySize);
	cmd.AddValue ("distance"  , "distance between two nodes"  , m_step);
	cmd.AddValue ("rate"      , "maximum trasmission rate", m_txrate);
	cmd.AddValue ("root"      , "Mac address of root mesh point in HWMP", m_root);
	cmd.AddValue ("channels"  , "Use different frequency channels for different interfaces.[0]", m_chan);
	cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point.[1]", m_nIfaces);
	//cmd.AddValue ("showRtable", "Show HWMP Routing tables.[0]", m_showRtable);
	cmd.AddValue ("traceNode" , "trace Node's number", m_traceNum);
        cmd.AddValue ("nodeSpeed", "speed of the each node", nodeSpeed);
ns3::PacketMetadata::Enable ();
	cmd.Parse (argc, argv);
}

void
MeshDot11sSim::CreateTopologyNodes ()
{
	NS_LOG_FUNCTION(this);

	// Create mesh nodes

	n = new Ptr<Node> [m_xSize*m_ySize];
	for (uint32_t i = 0; i < m_xSize*m_ySize; ++i) {
		n[i] = CreateObject<Node> ();
		//n[i]->AggregateObject (CreateObject<ConstantPositionMobilityModel> ());
		meshNodes.Add (n[i]);
	}

  

	MobilityHelper mobility;

  int64_t streamIndex = 0; // используется для обеспечения постоянной мобильности в 

  ObjectFactory posMesh;
  posMesh.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  posMesh.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  posMesh.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));

  Ptr<PositionAllocator> taPositionAlloc = posMesh.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);


  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));


  mobility.SetPositionAllocator (taPositionAlloc);
	//mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (meshNodes);
  streamIndex += mobility.AssignStreams (meshNodes, streamIndex);
  NS_UNUSED (streamIndex); // С этого момента streamIndex не используется
	
// Create BGR nodes
	bgr = CreateObject<Node> ();

	//move model
	ListPositionAllocator pos;
	Vector3D wiredPoint (0.0, 0.0, 0.0);
	pos.Add(wiredPoint);
	mobility.SetPositionAllocator(&pos);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(bgr);


}



void
MeshDot11sSim::ConfigureMeshLayer ()
{
	NS_LOG_FUNCTION(this);

	// Configure YansWifiChannel
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
//настройка под ESP32
//макс мощность 802.11b DSSS 1 Mbps 19.5 дБм , но тут 802.11а и OFDM
	phy.Set ("EnergyDetectionThreshold", DoubleValue (-89.0) );//пороговая чувствительность
	phy.Set ("CcaMode1Threshold"       , DoubleValue (-62.0) );
	phy.Set ("TxGain"                  , DoubleValue (0.0) );//было 1
	phy.Set ("RxGain"                  , DoubleValue (0.0) );//было 1
	phy.Set ("TxPowerStart"            , DoubleValue (m_txPowerStart) );
	phy.Set ("TxPowerLevels"           , UintegerValue (m_txPowerLevels) );
	phy.Set ("TxPowerEnd"              , DoubleValue (m_txPowerEnd) );
	phy.Set ("RxNoiseFigure"           , DoubleValue (7.0) );

	YansWifiChannelHelper channel;
	channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	channel.AddPropagationLoss  ("ns3::LogDistancePropagationLossModel", "Exponent", StringValue ("2.7"));
	phy.SetChannel (channel.Create ());

	// Configure the parameters of the Peer Link
	Config::SetDefault ("ns3::dot11s::PeerLink::MaxBeaconLoss"   , UintegerValue (20));
	Config::SetDefault ("ns3::dot11s::PeerLink::MaxRetries"      , UintegerValue (10));//4
	Config::SetDefault ("ns3::dot11s::PeerLink::MaxPacketFailure", UintegerValue (10));//5

	// Configure the parameters of the Peer Management Protocol
	Config::SetDefault ("ns3::dot11s::PeerManagementProtocol::EnableBeaconCollisionAvoidance", BooleanValue (false));

	// Configure the parameters of the HWMP -------------------------------------------------
	// set Max Queue Length
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::MaxQueueSize", UintegerValue (256));
	// set Lifetime of reactive routing information
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactivePathTimeout", TimeValue (Seconds (100)));
	// set Lifetime of poractive routing information
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactiveRootTimeout", TimeValue (Seconds (100)));
	// set Maximum number of retries before we suppose the destination to be unreachable
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPmaxPREQretries", UintegerValue (10));//5
	// set Maximum number of PREQ receivers, when we send a PREQ as a chain of unicasts
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastPreqThreshold", UintegerValue (10));//10
	// set Maximum number of broadcast receivers, when we send a broadcast as a chain of unicasts
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastDataThreshold", UintegerValue (10));//5
	// установить флаг "только для HWMP"
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::DoFlag", BooleanValue (true));//true
	// установить флаг "подтверди прием и передай дальше"
	Config::SetDefault ("ns3::dot11s::HwmpProtocol::RfFlag", BooleanValue (true));

	// Stack installer creates all protocols and install them to mesh point device
	mesh = MeshHelper::Default ();

/*
esp32 стандарты передачи
802.11b DSSS 1 Mbps, POUT = +19.5 dBm TxCurrent = 240 mA
802.11g OFDM 54 Mbps, POUT = +16 dBm TxCurrent = 190 mA
802.11n OFDM MCS7, POUT = +14 dBm TxCurrent = 180 mA
802.11b/g/n RxCurrent = 95~100 mA
*/
	mesh.SetStandard (WIFI_PHY_STANDARD_80211a);//a
	if (!Mac48Address (m_root.c_str ()).IsBroadcast ()) 
        {
		mesh.SetStackInstaller ("ns3::Dot11sStack", "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
	} 
        else 
        {
		mesh.SetStackInstaller ("ns3::Dot11sStack");
	}

	mesh.SetMacType ("RandomStart", TimeValue (Seconds(m_randomStart)));
	/*mesh.SetRemoteStationManager ( "ns3::ConstantRateWifiManager",
		                        "DataMode", StringValue ("OfdmRate6Mbps"),
		                        "RtsCtsThreshold", UintegerValue (2500));
*/

  mesh.SetRemoteStationManager ("ns3::AparfWifiManager", "DefaultTxPowerLevel", UintegerValue (m_txPower),"RtsCtsThreshold", UintegerValue (2500));

	// Set number of interfaces - default is single-interface mesh point
	mesh.SetNumberOfInterfaces (m_nIfaces);

	//If multiple channels is activated
	if (m_chan)
		mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
	else
		mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);

	// Install protocols and return container if MeshPointDevices
	meshDevices = mesh.Install (phy, meshNodes);
  

	//connect the mesh to the Internet---------------------------------------
	PointToPointHelper p2p;
	p2p.SetDeviceAttribute  ("DataRate", StringValue ("5Mbps"));
	p2p.SetChannelAttribute ("Delay"   , StringValue ("2ms"));

	// set portal node(n[0])
	p2pDevices = p2p.Install (bgr, n[0]);

   if (m_tracing)
     {
       phy.EnablePcapAll (std::string (std::string(PROG_DIR)+"mp-"));

       AsciiTraceHelper ascii;
       phy.EnableAsciiAll (ascii.CreateFileStream (std::string(PROG_DIR)+"ascii_trace.tr"));
     }

}

void
MeshDot11sSim::InstallInternetStack ()
{
	NS_LOG_FUNCTION(this);

	//Install the internet protocol stack on all nodes
	InternetStackHelper internetStack;
	internetStack.InstallAll();

	//Assign IP addresses to the devices interfaces
	Ipv4AddressHelper address;
	address.SetBase ("192.168.1.0", "255.255.255.0");
	address.Assign (p2pDevices);

	address.SetBase ("10.10.1.0", "255.255.255.0");
	address.Assign (meshDevices);

// установить статический маршрут от EGR до ячеистой сети (10.10.1.0/24) через узел портала (n [0]).
	Ipv4StaticRoutingHelper staticRouting;
	Ptr<Ipv4StaticRouting> bgrStatic = staticRouting.GetStaticRouting (bgr->GetObject<Ipv4> ());
	bgrStatic->AddNetworkRouteTo (Ipv4Address ("10.10.1.0"),
		Ipv4Mask ("255.255.255.0"), Ipv4Address ("192.168.1.2"), 1);

// установить маршрут по умолчанию для внешних сетей для каждого узла сетки
// 10.10.1.1 (n [0]) является узлом портала внешнего маршрутизатора шлюза Интернета
	Ipv4Address gateway ("10.10.1.1");
	Ptr<Ipv4StaticRouting> meshStatic;
	for (uint32_t i = 1; i < m_xSize*m_ySize; ++i) {
		meshStatic = staticRouting.GetStaticRouting (n[i]->GetObject<Ipv4> ());
		meshStatic->SetDefaultRoute(gateway, 1);
	}
}

void
MeshDot11sSim::SetUpTcpApplication()
{
	NS_LOG_FUNCTION(this);

//установка источников сигналов
	for (uint16_t i = 1; i < m_xSize*m_ySize; ++i)//источником может быть и сам шлюз!
        {
	// получить IP-адрес узла n в качестве адреса источника
	Ipv4InterfaceAddress sAdr = n[i]->GetObject <Ipv4> ()->GetAddress(1, 0);
	Address srcAddr (InetSocketAddress (sAdr.GetLocal(), 8000+i));

	// получить IP-адрес узла m в качестве адреса назначения
	Ipv4InterfaceAddress dAdr = bgr->GetObject <Ipv4> ()->GetAddress(1, 0);
	Address remoteAddr (InetSocketAddress (dAdr.GetLocal(), 8000+i));

	NS_LOG_UNCOND("Set TCP flow: " << sAdr.GetLocal() << " --> " << dAdr.GetLocal());

	OnOffHelper onOff = OnOffHelper ("ns3::TcpSocketFactory", Address());//ns3::TcpSocketFactory

	onOff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
	onOff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
	onOff.SetAttribute ("DataRate", DataRateValue (m_txrate));
	onOff.SetAttribute ("Remote", AddressValue(remoteAddr));
        onOff.SetAttribute ("PacketSize", UintegerValue(m_packetSize));
        
	ApplicationContainer src = onOff.Install (n[i]);
	src.Start (Seconds (30.1));
	src.Stop  (Seconds (60.1));

        

	// Create an optional packet sink to receive these packets
	PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), 8000+i)));

	ApplicationContainer sink = sinkHelper.Install (bgr);
	sink.Start (Seconds (30.1));
	sink.Stop  (Seconds (60.1 ));
        }
 /*
   UdpEchoServerHelper echoServer (1000+i);//используемый порт, изначально было 9

   ApplicationContainer serverApps = echoServer.Install (bgr);//сервером является нулеая точка доступа

   serverApps.Start (Seconds (30.1));
   serverApps.Stop (Seconds (m_totalTime-20));

	Ipv4InterfaceAddress dAdr = bgr->GetObject <Ipv4> ()->GetAddress(1, 0);
   UdpEchoClientHelper echoClient (dAdr.GetLocal(), 1000+i);//первый параметр - IP-адрес удаленного эхо-сервера udp, второй - Номер порта удаленного эхо-сервера udp

   echoClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)(m_totalTime*(1/0.1))));//указываем сколько пакетов хотим отправить!!!автоматически вычисляется по формуле в скобках
   echoClient.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
   echoClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

//один клиент
   ApplicationContainer clientApps = echoClient.Install (n[i]);



   clientApps.Start (Seconds (30.1));
   clientApps.Stop (Seconds (m_totalTime-20));
   }*/
}

/////////////////////////////////////////////////////////////////////////
void
MeshDot11sSim::RouteDiscoveryTimeSniffer (std::string context, Time time)
{
	NS_LOG_FUNCTION(this);
	std::ostringstream os;
  os << PROG_DIR << "RouteDiscoveryTime.log";
std::ofstream f;
f.open(os.str(), std::ios::out| std::ios::app);
f << Simulator::Now ().GetSeconds () << " RouteDiscoveryTime  :" << time.GetSeconds() << std::endl;
	std::cout << Simulator::Now ().GetSeconds () << " RouteDiscoveryTime  :" << time.GetSeconds() << std::endl;
}

template <int node>
void RemainingEnergyTrace (double oldValue, double newValue)
{
	std::ostringstream os;
  os << PROG_DIR << "EnergyLogFile-" << node << ".log";
std::ofstream f;
f.open(os.str(), std::ios::out| std::ios::app);
  //static std::fstream f (ss.str ().c_str (), std::ios::out | std::ios::app);

  f << Simulator::Now ().GetSeconds () << " " << newValue/3600/3.7*1000 << std::endl;
}

template <int node>
void PhyStateTrace (std::string context, Time start, Time duration, WifiPhyState state)
{
  std::stringstream ss;
  ss << PROG_DIR << "PhyState-" << node << ".log";

  static std::fstream f (ss.str ().c_str (), std::ios::out);

  f << Simulator::Now ().GetSeconds () << "    state=" << state << " start=" << start << " duration=" << duration << std::endl;
}


template <int node>
void RateCallback (std::string path, DataRate oldRate, DataRate newRate, Mac48Address remoteAddress)
{
  std::stringstream ss;
  ss << PROG_DIR << "TxRateChange-" << node << ".log";

  std::fstream f;
f.open(ss.str().c_str (), std::ios::out| std::ios::app);

  f << Simulator::Now ().GetSeconds () << " " << remoteAddress << " Rate " <<  newRate  << std::endl;
}

template <int node>
void PowerCallback  (std::string path, double oldPower, double newPower, Mac48Address remoteAddress)
{
  std::stringstream ss;
  ss << PROG_DIR << "TxPowerChange-" << node <<".log";

  std::fstream f;
f.open(ss.str().c_str (), std::ios::out| std::ios::app);

  f << Simulator::Now ().GetSeconds () << " " << remoteAddress << " Power " <<  newPower  << std::endl;
}

///////////////////////////////////////////////////////////////


//подключает энергетическую модель на узлы, вызывается из RunSim
EnergySourceContainer
MeshDot11sSim::AttachEnergyModelToDevices()
{
// настраиваем источник энергии и вывод отчета
double initialEnergy_Joule=initialEnergy/1000*3600*voltage;//перевод мА*ч -> в Джоули

	BasicEnergySourceHelper basicSourceHelper;
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (initialEnergy_Joule));
        basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (voltage));
        basicSourceHelper.Set ("PeriodicEnergyUpdateInterval", TimeValue(Seconds(50)));//установить период обновления энергии >10 с

	EnergySourceContainer energySourceContainer = basicSourceHelper.Install (meshNodes);//источник энергии устанавливаеться на каждый узел

	// transmit at 0dBm = 17.4mA, receive mode = 19.7mA
	MeshRadioEnergyModelHelper radioEnergyHelper;
	radioEnergyHelper.Set ("TxCurrentA", DoubleValue (txCurrent));
	radioEnergyHelper.Set ("IdleCurrentA", DoubleValue (idleCurrent));
	radioEnergyHelper.Set ("RxCurrentA", DoubleValue (rxCurrent));
        radioEnergyHelper.Set ("SleepCurrentA",DoubleValue (sleepCurrent));

// вычисляем эффективность усилителя мощности (eta), предполагая, что предоставленное значение для тока передачи соответствует минимальному уровню мощности передачи
  double eta = DbmToW (m_txPower) / ((txCurrent - idleCurrent) * voltage);


// Настройте модель тока передачи для этого источника энергии. установка линейной модели передачи тока Wi-Fi 
  radioEnergyHelper.SetTxCurrentModel ("ns3::LinearWifiTxCurrentModel",
                                       "Voltage", DoubleValue (voltage),
                                       "IdleCurrent", DoubleValue (idleCurrent),
                                       "Eta", DoubleValue (eta));

	radioEnergyHelper.SetDepletionCallback(MakeCallback(&MeshDot11sSim::NotifyDrained, this));
	DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (meshDevices, energySourceContainer);//необходима для вызова печати энргетики, видать отвечает за создание "RemainingEnergy"

for (uint32_t i=0; i < meshNodes.GetN(); ++i)
{
	std::ostringstream os;
	os << PROG_DIR << "EnergyLogFile-"<< i <<".log";
	std::ofstream energyLogFile (os.str().c_str(), std::ios::out);// разрешить запись в поток
        energyLogFile << "Узел " << i << " локальный MAC адрес " << Mac48Address::ConvertFrom(meshNodes.Get(i)->GetDevice(0)->GetAddress()) << "\n\n";
          energyLogFile << "Заряд батареии = " << initialEnergy << " [мА*ч], = " << initialEnergy_Joule << " Дж\n";
          energyLogFile << "Напряжение питания = " << voltage << " [В]\n";
          energyLogFile << "Ток холостого хода = " << idleCurrent*1000 << " [мА]\n";
          energyLogFile << "Максимальный ток передачи = " << txCurrent*1000 << " [мА]\n";
          energyLogFile << "Максимальный ток приема = " << rxCurrent*1000 << " [мА]\n";
          energyLogFile << "Ток в режиме 'sleep' = " << sleepCurrent*1000 << " [мА]\n\n";

}

//подключаем источники трассировки для вывода значений RemainingEnergy и State
          energySourceContainer.Get (0)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<0>));
          energySourceContainer.Get (1)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<1>));
          energySourceContainer.Get (2)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<2>));
          energySourceContainer.Get (3)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<3>));
          energySourceContainer.Get (4)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<4>));
          energySourceContainer.Get (5)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<5>));
          energySourceContainer.Get (6)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<6>));
          energySourceContainer.Get (7)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<7>));
          energySourceContainer.Get (8)->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergyTrace<8>));



  Config::Connect ("/NodeList/0/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace<0>));
  Config::Connect ("/NodeList/1/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace<1>));
  Config::Connect ("/NodeList/2/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace<2>));
  Config::Connect ("/NodeList/3/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace<3>));
	return energySourceContainer;
}


void//функция, оповещающая о том, что энергия закончилась
MeshDot11sSim::NotifyDrained()
{
	std::cout << Simulator::Now ().GetSeconds () <<" Energy was Drained. Stop send.\n";
	m_energyDrained = true;
}

void
MeshDot11sSim::RunSim (int argc, char *argv[])
{
	NS_LOG_FUNCTION(this);

	Configure (argc, argv);

	// use ConfigStore
	// following default configuration are same to
	// ./waf --run "exp06-simpleMesh --ns3::ConfigStore::Mode=Save
	//	 --ns3::ConfigStore::Filename=config.txt"
	std::string cf = std::string(PROG_DIR) + "config.txt";
	Config::SetDefault ("ns3::ConfigStore::Filename", StringValue (cf));
	Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
	Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
	ConfigStore config;
  	config.ConfigureDefaults ();

	CreateTopologyNodes ();
	ConfigureMeshLayer();

//расчет энергетики
/*
берётся базовый аккумулятор (емкость батареи)2500мАч*(напряжение выхода батареи)3.7В = 9.25 Вт*ч
Тх ток=240мА
Rx ток=100мА

   1 Вт*ч = 3600 Дж
9.25 Вт*ч = 33300 Дж
*/

	EnergySourceContainer energySourceContainer = AttachEnergyModelToDevices(); 

	InstallInternetStack ();
        SetUpTcpApplication();


	// Install FlowMonitor on all nodes
	monitor = flowmon.InstallAll();
/*
	if(m_showRtable)
        {
		Simulator::Schedule (Seconds(m_totalTime), &MeshDot11sSim::showHwmpRoutingTables, this);
	}
*/
        Simulator::Schedule (Seconds(m_totalTime), &MeshDot11sSim::Report, this);



	std::string xf = std::string(PROG_DIR) + "Animation-mesh.xml";
	AnimationInterface anim (xf);
//заменить стандартное изображение точек на робота
std::string resoursePath;//адрес картинки для вставки вметсо узла
resoursePath = "/home/daniil/workspace_ns3/ns3.28/ns-3.28/scratch/robot.png";//полный путь до картинки
uint32_t namberOfNodes=m_xSize*m_ySize;
  for (uint32_t nodeId = 0; nodeId < namberOfNodes; ++nodeId)
  {
     anim.UpdateNodeImage(nodeId, anim.AddResource(resoursePath));
     anim.UpdateNodeSize(nodeId, 20, 20);
  }

  anim.EnablePacketMetadata (); //включить отображение метаданных
  //anim.EnableIpv4RouteTracking ("MeshTrace.xml", Seconds(30.1), Seconds(60.1), Seconds(1));
  anim.EnableWifiMacCounters (Seconds (0), Seconds (m_totalTime)); //Включить отслеживание счетчиков Wi-Fi Mac, таких как Tx, TxDrop, Rx, RxDrop.
  anim.EnableWifiPhyCounters (Seconds (0), Seconds (m_totalTime)); //Включить отслеживание счетчиков Wi-Fi, таких как TxDrop, RxDrop.
	config.ConfigureAttributes ();

	m_timeStart = clock();

	Config::Connect ("/NodeList/*/DeviceList/*/$ns3::dot11s::HwmpProtocol/RouteDiscoveryTime", MakeCallback(&MeshDot11sSim::RouteDiscoveryTimeSniffer, this));//RouteDiscoveryTime - в списке TraceSources протокола HWMP

  //std::string meshRemoteManager="ns3::ArfWifiManager";
  std::string meshRemoteManager="ns3::AparfWifiManager";
for (uint32_t i=0; i < meshNodes.GetN(); ++i)
{
  std::stringstream txP;//инициализация файлов отчета TxPowerChange-i.log
  txP << PROG_DIR << "TxPowerChange-" << i <<".log";
std::fstream fileTxP;
fileTxP.open(txP.str().c_str (), std::ios::out);
fileTxP << "Узел " << i << " локальный MAC адрес " << Mac48Address::ConvertFrom(meshNodes.Get(i)->GetDevice(0)->GetAddress()) << "\n\n";

  std::stringstream txR;//инициализация файлов отчета TxRateChange-i.log
  txR << PROG_DIR << "TxRateChange-" << i <<".log";
std::fstream fileTxR;
fileTxR.open(txR.str().c_str (), std::ios::out);
fileTxR << "Узел " << i << " локальный MAC адрес " << Mac48Address::ConvertFrom(meshNodes.Get(i)->GetDevice(0)->GetAddress()) << "\n\n";
}
//вывод изменения скорости передачи
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<0>));
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<1>));
  Config::Connect ("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<2>));
  Config::Connect ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<3>));
  Config::Connect ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<4>));
  Config::Connect ("/NodeList/5/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<5>));
  Config::Connect ("/NodeList/6/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<6>));
  Config::Connect ("/NodeList/7/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<7>));
  Config::Connect ("/NodeList/8/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/RateChange", MakeCallback (&RateCallback<8>));

//вывод изменения мощности передачи
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<0>));
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<1>));
  Config::Connect ("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<2>));
  Config::Connect ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<3>));
  Config::Connect ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<4>));
  Config::Connect ("/NodeList/5/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<5>));
  Config::Connect ("/NodeList/6/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<6>));
  Config::Connect ("/NodeList/7/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<7>));
  Config::Connect ("/NodeList/8/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + meshRemoteManager + "/PowerChange", MakeCallback (&PowerCallback<8>));


	Simulator::Stop (Seconds (m_totalTime));

	Simulator::Run ();

	FlowMonitoring();
}

void
MeshDot11sSim::FlowMonitoring()
{
	NS_LOG_FUNCTION(this);
	std::string flowmon_xml = std::string(PROG_DIR) + "mesh.flowmon.xml";
  monitor -> SerializeToXmlFile(flowmon_xml, true, true);// =имя или путь к выходному файлу, который будет создан; =если true, включите также гистограммы в вывод; =если true, включите также статистику по каждому зонду / потоку в вывод

	// Define variables to calculate the metrics
	uint32_t totaltxPackets = 0;
	uint32_t totalrxPackets = 0;
	double   totaltxbytes   = 0;
	double   totalrxbytes   = 0;
	double   totaldelay     = 0;
        double   totalJitter    = 0;//
	double   totalrxbitrate = 0;
	double   diffTx, diffRx;
	double   pdr_value, plr_value, rxbitrate_value, delay_value, jitter_value, txbitrate_value;
	double   pdr_total, plr_total, rxbitrate_total, delay_total, jitter_total;

//печатаем все результаты в файл 
	std::ostringstream os;
	os << PROG_DIR << "mesh-metrics"<<nodeSpeed<<".txt";
	std::ofstream of (os.str().c_str(), std::ios::out);
        of << "Скорость узлов " << nodeSpeed << "\n";
	of << "---------------------------------------------\n";
        std::cout << "---------------------------------------------\n";
	//Print per flow statistics
	monitor->CheckForLostPackets ();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
	std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	uint32_t k = 0;

	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) 
        {
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);

		diffTx = i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds();

		diffRx = i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstRxPacket.GetSeconds();

		pdr_value = (double) i->second.rxPackets / (double) i->second.txPackets * 100;

		txbitrate_value = (double) i->second.txPackets * m_packetSize * 8 / 1000 / diffTx;// делим на 1000, т.к. надписть "k" означает 1000, "ki" - 1024

                plr_value = ((double)i -> second.txPackets - (double)i -> second.rxPackets) / (double)i -> second.txPackets * 100;

		if (i->second.rxPackets != 0 || i->second.rxPackets != 1) 
                {
			rxbitrate_value = (double)i->second.rxPackets * m_packetSize * 8 / 1000 / diffRx;

			delay_value = (double) i->second.delaySum.GetSeconds() / (double) i->second.rxPackets;

                        jitter_value = (double) i->second.jitterSum.GetSeconds() / ((double) i->second.rxPackets - 1);
		} 
                else 
                {
			rxbitrate_value = 0;
			delay_value = 0;
                        jitter_value = 0;
		}

                // Нас интересуют только метрики потоков данных
		if ((!t.destinationAddress.IsSubnetDirectedBroadcast("255.255.255.0"))) {
			k++;

			// напечатать статистику для каждого потока в файл 
			of << "Flow " << k << " (" << t.sourceAddress << "{Port = " << t.sourcePort << "}" << " -> " << t.destinationAddress << "{Port = " << t.destinationPort << "} )\n";
                        of << "Передано пакетов: " << i -> second.txPackets << "\n";
                        of << "Передано : " << i -> second.txBytes << " байт\n";
                        of << "Принято пакетов: " << i -> second.rxPackets << "\n";
                        of << "Принято: " << i -> second.rxBytes << " байт\n";
                        of << "Потеряно пакетов: " << i -> second.lostPackets << "\n";
                        of << "Отброшено пакетов: " << i -> second.packetsDropped.size() << "\n\n";
			of << "PDR: " << pdr_value << " %\n";
                        of << "PLR: " << plr_value << " %\n";
			of << "Average delay: " << delay_value << " c\n";
                        of << "Avarage Jitter: " << jitter_value << " c\n";
			of << "Rx bitrate: " << rxbitrate_value << " кбит/с\n";
			of << "Tx bitrate: " << txbitrate_value << " кбит/с\n";
			of << "---------------------------------------------\n";

			// напечатать статистику для каждого потока в командную строку
			std::cout << "Flow " << k << " (" << t.sourceAddress << "{Port = " << t.sourcePort << "}" << " -> " << t.destinationAddress << "{Port = " << t.destinationPort << "} )\n";
                        std::cout << "Передано пакетов: " << i -> second.txPackets << "\n";
	std::cout << "Передано : " << i -> second.txBytes << " байт\n";
                        std::cout << "Принято пакетов: " << i -> second.rxPackets << "\n";
	std::cout << "Принято: " << i -> second.rxBytes << " байт\n";
                        std::cout << "Потеряно пакетов: " << i -> second.lostPackets << "\n";
                        std::cout << "Отброшено пакетов: " << i -> second.packetsDropped.size() << "\n\n";
			std::cout << "PDR: " << pdr_value << " %\n";
                        std::cout << "PLR: " << plr_value << " %\n";
			std::cout << "Average delay: " << delay_value << "s\n";
                        std::cout << "Avarage Jitter: " << jitter_value << "s\n";
			std::cout << "Rx bitrate: " << rxbitrate_value << " kbps\n";
			std::cout << "Tx bitrate: " << txbitrate_value << " kbps\n";
			std::cout << "---------------------------------------------\n";

			// Накапливаем для усреднения статистики
			totaltxPackets += i->second.txPackets;
			totaltxbytes += i->second.txBytes;
			totalrxPackets += i->second.rxPackets;
			totaldelay += i->second.delaySum.GetSeconds();
			totalrxbitrate += rxbitrate_value;
			totalrxbytes += i->second.rxBytes;
                        totalJitter += i->second.jitterSum.GetSeconds();
		}
	}

	// Average all nodes statistics
	if (totaltxPackets != 0){
		pdr_total = (double) totalrxPackets / (double) totaltxPackets * 100;
                plr_total = ((double) totaltxPackets - (double) totalrxPackets) / (double) totaltxPackets * 100;}
	else
		pdr_total = 0;

	if (totalrxPackets != 0 || totalrxPackets!=1) {
		rxbitrate_total = totalrxbitrate;
		delay_total = (double) totaldelay / (double) totalrxPackets;
                jitter_total = (double) totalJitter / ((double)totalrxPackets -1);
	} else {
		rxbitrate_total = 0;
		delay_total = 0;
                jitter_total = 0;
	}

	//print all nodes statistics
	NS_LOG_UNCOND ("Total PDR: " << pdr_total << " %");
	NS_LOG_UNCOND ("Total PLR: " << plr_total << " %");
	NS_LOG_UNCOND ("Total Rx bitrate: " << rxbitrate_total << " kbps");
	NS_LOG_UNCOND ("Total Delay: " << delay_total << " s");
        NS_LOG_UNCOND ("Avarage Jitter: " << jitter_total << " s");

	of << "Total PDR: " << pdr_total << " %\n";
        of << "Total PLR: " << plr_total << " %\n";
        of << "Total Delay: " << delay_total << " s\n";
        of << "Total Jitter: " << jitter_total << " s\n";
	of << "Total Rx bitrate: " << rxbitrate_total << " kbps\n";

	of.close ();

	Simulator::Destroy ();

	m_timeEnd   = clock();
	m_timeTotal = (m_timeEnd - m_timeStart)/(double) CLOCKS_PER_SEC;

	NS_LOG_UNCOND ("\nSimulation time: " << m_timeTotal << "s");
}

void MeshDot11sSim::Report ()
{
	NS_LOG_FUNCTION(this);

   unsigned n (0);

   for (NetDeviceContainer::Iterator i = meshDevices.Begin (); i != meshDevices.End (); ++i, ++n)
     {
       std::ostringstream os;
       os << PROG_DIR << "mp-report-" << n << ".xml";
       std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
       std::ofstream of;
       of.open (os.str ().c_str ());
       if (!of.is_open ())
         {
           std::cerr << "Error: Can't open file " << os.str () << "\n";
           return;
         }
       mesh.Report (*i, of);
       of.close ();
     }
}
/*
void
MeshDot11sSim::showHwmpRoutingTables()
{
        NS_LOG_FUNCTION(this);

	for (uint32_t i = 0; i < m_xSize*m_ySize; ++i) {
	Ptr<NetDevice> ndev = n[i]->GetDevice(0);
	NS_ASSERT (ndev != 0);
	Ptr<MeshPointDevice> mdev = ndev->GetObject<MeshPointDevice>();
	NS_ASSERT (mdev != 0);
	Ptr<ns3::dot11s::HwmpProtocol> hwmp = mdev->GetObject<ns3::dot11s::HwmpProtocol> ();
	NS_ASSERT (hwmp != 0);
        Ptr<ns3::dot11s::HwmpRtable> rtable=hwmp->GetObject<ns3::dot11s::HwmpRtable>();
//rtable->LookupReactiveExpired(Mac48Address(meshNodes.Get(0)->GetDevice(0)->GetAddress()));
	//hwmp->Report(std::cout);//выводит в концоль часть данных из общего отчета, относящуюся к hwmp.
Mac48Address mac48adr=Mac48Address::ConvertFrom(meshNodes.Get(i)->GetDevice(0)->GetAddress());
//NS_LOG_UNCOND( rtable->LookupReactiveExpired(mac48adr));

//NS_LOG_UNCOND(hwmp->GetRoutingTable()->LookupReactiveExpired(Mac48Address("192.168.1.1")));
	//hwmp->ReportRtables(std::cout, m_xSize*m_ySize);
	}
}
*/

int main (int argc, char *argv[])
{
	MeshDot11sSim sim;
	
	sim.RunSim(argc, argv);

	return 0;

}
