#include <Wt/WServer>
#include <Wt/WResource>
#include <Wt/Http/Response>

using namespace Wt;

class RestGetHello : public Wt::WResource
{
   public:
       virtual ~RestGetHello(){}

   protected:
       virtual void handleRequest(const Wt::Http::Request &request,
Wt::Http::Response &response)
       {
           response.out() << "Hello World!\n";
       }
};

#if defined(WITH_WAPP)
class RestApplication : public WApplication
{
   public:
       RestApplication(const WEnvironment& env)
           : WApplication(env)
       {
           WServer::instance()->addResource(&getHello, "/hello");
       }

   private:
       RestGetHello getHello;
};

WApplication *createApplication(const WEnvironment& env)
{
   return new RestApplication(env);
}
#endif

int main(int argc, char **argv)
{
#if defined(WITH_WAPP)
   return Wt::WRun(argc, argv, &createApplication);
#else
   try {
       WServer server(argv[0]);

       server.setServerConfiguration(argc, argv);

       RestGetHello getHello;

       server.addResource(&getHello, "/hello");

       if (server.start()) {
           WServer::waitForShutdown();
           server.stop();
       }
   } catch (WServer::Exception& e) {
       std::cerr << e.what() << std::endl;
   } catch (std::exception &e) {
       std::cerr << "exception: " << e.what() << std::endl;
   }
#endif
}
