#include <camera.h>
#include <signal.h>

#include <atomic>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>

#include "agent_runner.h"
#include "always_play_gc.h"
#include "async.h"
#include "ball_tracker.h"
#include "camera_pub_sub.h"
#include "game_controller.h"
#include "image.h"
#include "localization.h"
#include "logging.h"
#include "multi_target_tracker_pub_sub.h"
#include "near_obstacle_tracker.h"
#include "odometer_processor.h"
#include "parameter_tuner/parameter_tuner.h"
#include "sensor/sensor.h"
#include "shootorder.h"
#include "soccerfield/soccerfield.h"
#include "soccerfield/soccerfielddefinitions.h"
#include "tc_network.h"
#include "team_strategy.h"
#include "team_strategy_factory.h"
#include "tts.h"
#include "vision/htwk_vision.h"
#include "vision_pub_sub.h"
#include "walkcustomorder.h"
#include "walkrelativeorder.h"
#include "walktopositionorder.h"

#ifndef SIMULATION_MODE

#else
#include "sim_mock.h"
#endif

#if ROBOT_MODEL_K1
#include "motion_connector_k1.h"
#elif ROBOT_MODEL_T1
#include "motion_connector_t1.h"
#endif

std::atomic<bool> running{true};

extern htwk::Channel<std::shared_ptr<Image>> images;

void signalHandler(int signum) {
    if (signum == SIGINT) {
        running = false;
    }
}

int vision_loop_count = 0;
int64_t last_loop_log_time = time_us();

uint64_t frame_number = 0;

void visionLoop(TeamIdx team_id, PlayerIdx player_id, const std::string& strategy_name) {
    LOG_F(INFO, "Starting vision loop");
    htwk::HTWKVision vision_htwk;
    ThreadPool thread_pool("MainVision");
    htwk::OdometerProcessor odometer_processor;
    Localization localization(player_id);
    BallTracker ball_tracker;
    htwk::NearObstacleTracker near_obstacle_tracker;
    htwk::RelRobotsTracker robot_tracker;
    std::unique_ptr<TeamStrategy> team_strategy =
            TeamStrategyFactory::create(strategy_name, team_id, player_id);
    AgentRunner agent_runner("competition");

    auto image_subscriber = images.create_subscriber();
    auto opponent_hypotheses_subscriber = opponent_hypotheses_channel.create_subscriber();

    int64_t image_subscriber_time_acc = 0;
    int64_t vision_htwk_time_acc = 0;
    int64_t near_obstacle_tracker_time_acc = 0;
    int64_t odometer_processor_time_acc = 0;
    int64_t localization_time_acc = 0;
    int64_t ball_tracker_time_acc = 0;
    int64_t team_strategy_time_acc = 0;
    int64_t agent_runner_time_acc = 0;
    int64_t motion_connector_time_acc = 0;
    int64_t robot_tracker_time_acc = 0;
    int64_t total_loop_time_acc = 0;

    while (running) {
        TaskScheduler scheduler(&thread_pool);

        int64_t start_time = time_us();
        std::shared_ptr<Image> img = image_subscriber.next();
        int64_t image_available_time = time_us();
        image_subscriber_time_acc += image_available_time - start_time;

        auto vision = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    vision_htwk.proceed(img);
                    vision_htwk_time_acc += time_us() - start_time;
                },
                {});

        auto near_obstacle = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    near_obstacle_tracker.proceed(img->timestamp_us);
                    near_obstacle_tracker_time_acc += time_us() - start_time;
                },
                {vision});

        auto odometer = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    odometer_processor.proceed();
                    odometer_processor_time_acc += time_us() - start_time;
                },
                {});

        auto localization_task = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    localization.proceed(*img->cam_pose_ptr, img->timestamp_us);
                    localization_time_acc += time_us() - start_time;
                },
                {vision});

        auto ball_tracker_task = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    ball_tracker.proceed(*img);
                    ball_tracker_time_acc += time_us() - start_time;
                },
                {vision});

        auto robot_tracker_task = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    std::vector<htwk::TrackedObject> percepts;
                    for (const auto& hypo : opponent_hypotheses_subscriber.latest()) {
                        // TODO: Determine which robot is opponent vs us.
                        if (auto pRobot = CamUtils::project(hypo.point(), *img->cam_pose_ptr)) {
                            // Filter out the left shoulder.
                            if (pRobot->x < 0 && pRobot->y < 0.35)
                                continue;
                            percepts.emplace_back(*pRobot, hypo.prob, 0.5f, 0, hypo.point());
                        }
                    }
                    // TODO: add ready_for_action everywhere.
                    robot_tracker.proceed(percepts, img->timestamp_us, localization.getPosition(),
                                          img->cam_pose_ptr->get_translation().z, true);
                    robot_tracker_time_acc += time_us() - start_time;
                },
                {vision, localization_task});

        auto team_strategy_task = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    team_strategy->proceed();
                    team_strategy_time_acc += time_us() - start_time;
                },
                {localization_task, ball_tracker_task, robot_tracker_task});

        auto agent_runner_task = scheduler.addTask(
                [&]() {
                    int64_t start_time = time_us();
                    agent_runner.proceed();
                    agent_runner_time_acc += time_us() - start_time;
                },
                {team_strategy_task});
        scheduler.run();

        frame_number++;

        total_loop_time_acc += time_us() - image_available_time;
        {
            vision_loop_count++;
            int64_t now = time_us();
            if (now - last_loop_log_time > 1'000'000) {
                if (vision_loop_count > 0) {
                    LOG_F(1,
                          "avg times (ms): total: %.2f, img: %.2f, vis: %.2f, near_obs: %.2f, loc: "
                          "%.2f, ball: %.2f, robot: %.2f, strat: %.2f, agent: %.2f, fps: %d",
                          total_loop_time_acc / (float)vision_loop_count / 1000.f,
                          image_subscriber_time_acc / (float)vision_loop_count / 1000.f,
                          vision_htwk_time_acc / (float)vision_loop_count / 1000.f,
                          near_obstacle_tracker_time_acc / (float)vision_loop_count / 1000.f,
                          localization_time_acc / (float)vision_loop_count / 1000.f,
                          ball_tracker_time_acc / (float)vision_loop_count / 1000.f,
                          robot_tracker_time_acc / (float)vision_loop_count / 1000.f,
                          team_strategy_time_acc / (float)vision_loop_count / 1000.f,
                          agent_runner_time_acc / (float)vision_loop_count / 1000.f,
                          vision_loop_count);
                }
                last_loop_log_time += 1'000'000;
                vision_loop_count = 0;
                image_subscriber_time_acc = 0;
                vision_htwk_time_acc = 0;
                near_obstacle_tracker_time_acc = 0;
                localization_time_acc = 0;
                ball_tracker_time_acc = 0;
                robot_tracker_time_acc = 0;
                team_strategy_time_acc = 0;
                agent_runner_time_acc = 0;
                motion_connector_time_acc = 0;
                total_loop_time_acc = 0;
            }
        }
    }
    LOG_F(INFO, "Vision loop ended");
}

void motionLoop(PlayerIdx player_id) {
    LOG_F(INFO, "Starting motion loop");
    MotionConnector motion_connector(player_id);
    int64_t time = time_us();
    int64_t missed_loops = 0;
    while (running) {
        motion_connector.proceed();
        do {
#ifdef ROBOT_MODEL_K1
            time += 5_ms;
#else
            time += 2_ms;
#endif
            missed_loops++;
        } while (time < time_us());
        missed_loops--;
        std::this_thread::sleep_until(
                std::chrono::system_clock::time_point(std::chrono::microseconds(time)));
        if (missed_loops > 100) {
            LOG_F(1, "Missed 100 motion loops");
            missed_loops = 0;
        }
    }
}

int main(int argc, char** argv) {
    // debug log all args
    //  LOG_F(INFO, "Command line arguments:");
    //  for (int i = 0; i < argc; i++) {
    //      LOG_F(INFO, "argv[%d]: %s", i, argv[i]);
    //  }

    // default values
    int team_id = 44;
    int player_id = 1;
    bool real_gc = false;
    std::string strategy_name = "robocup";
    std::string rerun_ip;

    // read jersey number from file if possible
    int jerseyNumber = 2;  // default
    std::string jerseyNumberFile = "/home/booster/jerseynumber.txt";
    if (boost::filesystem::exists(jerseyNumberFile)) {
        std::ifstream jerseyNumberStream(jerseyNumberFile);
        jerseyNumberStream >> jerseyNumber;  // set jersey number from file
        jerseyNumberStream.close();

        // switch back to default if none or wrong jersey number found
        if (jerseyNumber < 1) {
            jerseyNumber = 2;
        }
    }

    player_id = jerseyNumber - 1;

    // defined program options
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()("help,h", "shows you this page")(
            "team,t", boost::program_options::value<int>(&team_id)->default_value(team_id),
            "team ID")("player,p",
                       boost::program_options::value<int>(&player_id)->default_value(player_id),
                       "player ID")(
            "real_gamecontroller,gc",
            boost::program_options::value<bool>(&real_gc)->default_value(real_gc),
            "use real game controller")("strategy,s",
                                        boost::program_options::value<std::string>(&strategy_name)
                                                ->default_value(strategy_name),
                                        "name of the strategy to use")(
            "rerun_ip",
            boost::program_options::value<std::string>(&rerun_ip)->default_value(rerun_ip),
            "IP address for rerun stream (empty for local)");

    boost::program_options::variables_map vm;
    try {
        boost::program_options::command_line_parser parser(argc, argv);
        parser.options(desc).style(boost::program_options::command_line_style::default_style);
        boost::program_options::store(parser.run(), vm);
        boost::program_options::notify(vm);
    } catch (const boost::program_options::error& ex) {
        std::cerr << "Error parsing command line arguments: " << ex.what() << "\n";
        return 1;
    }

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 0;
    }
    LOG_F(INFO, "Hello boosted HTWK");
    say("Starting firmware");

    // log starting parameters
    LOG_F(INFO, "Starting firmware with the following parameters:");
    LOG_F(INFO, "Team ID: %d", team_id);
    LOG_F(INFO, "Player ID: %d", player_id);
    LOG_F(INFO, "Real GameController: %s", real_gc ? "true" : "false");
    LOG_F(INFO, "Strategy: %s", strategy_name.c_str());
    LOG_F(INFO, "Rerun IP: %s", rerun_ip.c_str());

    // change this in salvador
    SoccerFieldInstance soccerfield = SoccerFieldInstance(getSoccerFieldAbuDhabi(), 0);
    SoccerField::set_instance(soccerfield);
    htwk::log_soccer_field();

#ifdef ROBOT_MODEL_K1
    booster::robot::ChannelFactory::Instance()->Init(0, "127.0.0.1");
#elif ROBOT_MODEL_T1
#if ROBOT_SUBMODEL_T1_ROBOCUP_VERSION
    booster::robot::ChannelFactory::Instance()->Init(0, "127.0.0.1");
#else
    booster::robot::ChannelFactory::Instance()->Init(0, "192.168.10.102");
#endif
#endif
    signal(SIGINT, signalHandler);

#ifndef SIMULATION_MODE
    std::unique_ptr<GCInterface> gc_interface;
    if (real_gc) {
        gc_interface = std::make_unique<GameController>(team_id, player_id);
    } else {
        gc_interface = std::make_unique<AlwaysPlayGC>(team_id, player_id);
    }
    std::unique_ptr<htwk::Camera> camera = htwk::choose_camera();
    camera->start();

    htwk::Sensor sensor;
#else
    SimMock sim_mock;
    sim_mock.start();
#endif

    TeamComNetwork team_com_network(team_id);

    std::thread vision_thread =
            named_thread("vision", visionLoop, team_id, player_id, strategy_name);
    std::thread motion_thread = named_thread("motion", motionLoop, player_id);

    // Wait for Ctrl+C
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_F(INFO, "Starting shutdown sequence");

    // First stop the camera to prevent threads from blocking on new images
    camera->stop();

    vision_thread.join();
    LOG_F(INFO, "Vision thread joined");
    motion_thread.join();
    LOG_F(INFO, "Motion thread joined");
    LOG_F(INFO, "All threads joined");
    return 0;
}
