/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// std
#include <string>

// yarp
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    // BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    // BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    // Vector cogL,cogR;
    // bool okL,okR;

    int startup_gaze_context_id;
    int startup_cart_context_id;

    // bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    // {
    //     int xMean=0;
    //     int yMean=0;
    //     int ct=0;

    //     for (int x=0; x<img.width(); x++)
    //     {
    //         for (int y=0; y<img.height(); y++)
    //         {
    //             PixelRgb &pixel=img.pixel(x,y);
    //             if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
    //             {
    //                 xMean+=x;
    //                 yMean+=y;
    //                 ct++;
    //             }
    //         }
    //     }

    //     if (ct>0)
    //     {
    //         cog.resize(2);
    //         cog[0]=xMean/ct;
    //         cog[1]=yMean/ct;
    //         return true;
    //     }
    //     else
    //         return false;
    // }

    // Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    // {
    //     // get the position of the blue ball within the image planes
    //     Vector px_left(2), px_right(3);
        
    //     px_left = cogL;
    //     px_right = cogR;

    //     // request triangulation to find the cartesian position of the blue ball
    //     Vector x;
    //     igaze->triangulate3DPoint(px_left,px_right,x);
        
    //     return x;
    // }

    void fixate(const Vector &x)
    {
        // request motion and wait for completion
        igaze->lookAtFixationPointSync(x);
        igaze->waitMotionDone();
    }

    Vector computeHandOrientation()
    {
        // given the reference frame convention for the hands of iCub
        // it is required to have the x-axis attached to the center of the
        // palm pointing forward, the y-axis pointing downward and
        // the z-axis pointing lefttward
        //
        // one solution to obtain the final attitude w.r.t to the waist frame
        // is to compose a rotation of pi about the z-axis and a rotation
        // of -pi/2 about the x-axis (after the first rotation)

        Vector att_z(4), att_x(4);
        att_z[0] = 0.0;
        att_z[1] = 0.0;
        att_z[2] = 1.0;
        att_z[3] = +M_PI;

        att_x[0] = 1.0;
        att_x[1] = 0.0;
        att_x[2] = 0.0;
        att_x[3] = -M_PI/2.0;

        // convert to dcm
        Matrix Rz = yarp::math::axis2dcm(att_z);
        Matrix Rx = yarp::math::axis2dcm(att_x);

        // compose in current axes
        Matrix R = Rz*Rx;

        // convert back to axis
        Vector att = yarp::math::dcm2axis(R);
        
        return att;
    }

    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        // in order to avoid hitting the ball or the table surface
        // it is better to move it a little rightward
        // (i.e. more positive on waist y-axis)
        // and upward (more positive on waist z-axis)
        
        // Vector x_shifted = x;
        // x_shifted[1] += HAND_APPROACH_Y_TOLERANCE;
        // x_shifted[2] += HAND_APPROACH_Z_TOLERANCE;

        // request pose to the cartesian interface
        iarm->goToPoseSync(x, o);

        // wait for motion completion
        iarm->waitMotionDone();

    }

    void roll(const Vector &x, const Vector &o)
    {
        // desired final pose
        Vector x_d = x;
        // use the same z coordinate as in the approach phase
        // x_d[2] += HAND_APPROACH_Z_TOLERANCE;

        // final pose in the negative waist y-direction
        x_d[1] -= 0.01;

        // store the current context because we are going
        // to change the trajectory time
        int context_id;
        iarm->storeContext(&context_id);

        // set trajectory time
        iarm->setTrajTime(0.3);

        // request pose to the cartesian interface
        iarm->goToPoseSync(x_d, o);

        // wait for motion completion
        iarm->waitMotionDone(0.04);

        // restore the context
        iarm->restoreContext(context_id);

    }

    // void look_down()
    // {
    //     // consider a fixation point on the table surface
    //     Vector fp(3,0.0);
    //     // x-component, look in the forward direction
    //     fp[0] = -0.30;    
    //     // y-component is 0, pitch only required 
    //     // z-component is 0, the table surface is below the waist

    //     // request motion and wait for completion
    //     igaze->lookAtFixationPointSync(fp);
    //     igaze->waitMotionDone();
        
    // }

    void make_it_roll()//(const Vector &cogL, const Vector &cogR)
    {
        // yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";

        // Vector x=retrieveTarget3D(cogL,cogR);
        // yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        // fixate(x);
        // yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // Vector o=computeHandOrientation();
        // yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        // yarp::sig::Vector x(3, 0.0);
        // x[0] = 0.3;
        // x[1] = 0.3;
        // x[2] = 0.3;        
        // approachTargetWithHand(x,o);
        // yInfo()<<"approached";

        // roll(x,o);
        // yInfo()<<"roll!";
    }

public:
    bool configure(ResourceFinder &rf)
    {
        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/icubSim/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        // open the iCartesianControl view
        drvArm.view(iarm);

        // store the current context so that
        // it can be restored when the modules closes
        iarm->storeContext(&startup_cart_context_id);

        // set trajectory time
        iarm->setTrajTime(1.0);

        // use also the torso
        Vector newDoF, curDoF;
        iarm->getDOF(curDoF);
        newDoF = curDoF;

        newDoF[0] = 1;
        newDoF[1] = 1;
        newDoF[2] = 1;

        iarm->setDOF(newDoF,curDoF);

        // configure options for drvGaze
        // Property optGaze;
        // optGaze.put("device", "gazecontrollerclient");
        // optGaze.put("remote", "/iKinGazeCtrl");
        // optGaze.put("local", "/tracker/gaze");

        // open the driver
        // if (!drvGaze.open(optGaze))
        // {
        //     yError()<<"Unable to open the Gaze Controller";
        //     return false;
        // }

        // open the iGazeController view
        // drvGaze.view(igaze);

        // store the current context so that
        // it can be restored when the modules closes
        // igaze->storeContext(&startup_gaze_context_id);

        // set trajectory time
        // igaze->setNeckTrajTime(1.0);

        // imgLPortIn.open("/imgL:i");
        // imgRPortIn.open("/imgR:i");

        // imgLPortOut.open("/imgL:o");
        // imgRPortOut.open("/imgR:o");

        // set default values for okL and okR
        // okL = okR = false;

        rpcPort.open("/service");
        attach(rpcPort);

        return true;
    }

    bool interruptModule()
    {
        // imgLPortIn.interrupt();
        // imgRPortIn.interrupt();
        return true;
    }

    bool close()
    {
        // stop the gaze controller for safety reason
        // igaze->stopControl();

        // restore the gaze controller context
        // igaze->restoreContext(startup_gaze_context_id);

        // stop the cartesian controller for safety reason
        iarm->stopControl();

        // restore the cartesian controller context
        iarm->restoreContext(startup_cart_context_id);

        drvArm.close();
        // drvGaze.close();
        // imgLPortIn.close();
        // imgRPortIn.close();
        // imgLPortOut.close();
        // imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- test");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            // look_down();
            // // we assume the robot is not moving now
            // reply.addString("ack");
            // reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="test")
        {
	    Vector o = computeHandOrientation();
	    Vector x(3, 0.0);
	    x[0] = -0.38;
	    x[1] = 0.152;
	    x[2] = -0.046;
	    approachTargetWithHand(x, o);
	    
            // go only if the ball has been detected
            // bool go=false;
            // mutex.lock();
            // go = okL && okR;
            // Vector cogL = this->cogL;
            // Vector cogR = this->cogR;
            // mutex.unlock();

            // if (go)
            // {
            // make_it_roll(cogL,cogR);
                // we assume the robot is not moving now
                // reply.addString("ack");
                // reply.addString("Yeah! I've made it roll like a charm!");
            // }
            // else
            // {
            //     reply.addString("nack");
            //     reply.addString("I don't see any object!");
            // }
            reply.addString("ok!");	    
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    double getPeriod()
    {
        // return 0.0;     // sync upon incoming images
        return 1.0;
    }

    bool updateModule()
    {
	if(isStopping())
	    return false;
        // // get fresh images
        // ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        // ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // // interrupt sequence detected
        // if ((imgL==NULL) || (imgR==NULL))
        //     return false;

        // // compute the center-of-mass of pixels of our color
        // mutex.lock();
        // okL=getCOG(*imgL,cogL);
        // okR=getCOG(*imgR,cogR);
        // mutex.unlock();

        // PixelRgb color;
        // color.r=255; color.g=0; color.b=0;

        // if (okL)
        //     draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        // if (okR)
        //     draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        // imgLPortOut.prepare()=*imgL;
        // imgRPortOut.prepare()=*imgR;

        // imgLPortOut.write();
        // imgRPortOut.write();

        return true;
    }
};

int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}

