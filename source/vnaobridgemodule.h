#ifndef INCLUDE_VNAOBRIDGEMODULE_H
#define INCLUDE_VNAOBRIDGEMODULE_H


// in order to receive events from naoqi (e.g. text-to-speech events) we need to register a "module" object
class VNaoBridge;
class ALVNaoBridgeModule : public AL::ALModule {
private:
	AL::ALMemoryProxy memoryProxy;
	VNaoBridge* bridge;

public:
	ALVNaoBridgeModule(boost::shared_ptr<AL::ALBroker> broker, VNaoBridge* bridge) : AL::ALModule(broker, "ALVNaoBridgeModule"), memoryProxy(getParentBroker()), bridge(bridge) {
		setModuleDescription("");

		functionName("callback", getName(), "");
		BIND_METHOD(ALVNaoBridgeModule::callback);
	}

	virtual ~ALVNaoBridgeModule() {
		memoryProxy.unsubscribeToEvent("ALTextToSpeech/CurrentSentence", "ALVNaoBridgeModule");
	}

	// Will be called at module startup
	virtual void init() {
		try {
			memoryProxy.subscribeToEvent("ALTextToSpeech/CurrentSentence", "ALVNaoBridgeModule", "callback");
		}
		catch(const AL::ALError& e) {
			std::cerr << e.what() << std::endl;
		}
	}

	void callback(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg);
};


#endif
