#include <libgen.h>         // dirname
#include <unistd.h>         // readlink
#include <linux/limits.h>   // PATH_MAX
#include <iostream>

#include <boost/filesystem.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/crba.hpp>

namespace controller_ns{

  class algo{
    public:
      void print();
    private:
      void calculate_Jacobians();
  };

  void algo::print(){
    calculate_Jacobians();
  }

  void algo::calculate_Jacobians(){
    pinocchio::Model model_complete; // Modelo con los dos brazos accionable
    pinocchio::Model model;          // Modelo con solo un brazo accionable
    pinocchio::Data data;
    Eigen::VectorXd dot_q(7);
    for(int i = 0; i < 7; i++){
      dot_q(i) = i;
    }
    // std::string end_effector_link = "wrist_right_ft_tool_link";
    std::string end_effector_link = "arm_right_7_link";


    // Se recupera la ruta de la descripcion URDF
    // char result[PATH_MAX];
    // ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    // std::string urdf_path = std::string(dirname(result)) + std::string("/../talos_arms.urdf");
    std::string urdf_path = std::string("/home/adrii/talos_public_ws/src/talos_astronaut/astronaut/urdfs/astronaut_pinocchio_no_hands.urdf");

    // Se carga el modelo del robot en pinocchio
    std::cout << "Cargando archivo: " << urdf_path << "\n";
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_complete);
    std::cout << "Modelo cargado correctamente. Nombre del robot: " << model_complete.name << '\n';

    // Se crea la lista de articulaciones que se van a bloquear
    std::vector<std::string> articulaciones_bloqueadas{
        "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", 
        "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint", "torso_1_joint", "torso_2_joint", 
        "head_1_joint", "head_2_joint"
    };

    // , "hand_right_thumb_joint", "hand_right_index_joint", "hand_right_mrl_joint", 
    //     "hand_left_thumb_joint", "hand_left_index_joint", "hand_left_mrl_joint"
    // Se obtienen los ids de esas articulaciones
    std::vector<pinocchio::JointIndex> articulaciones_bloqueadas_id;
    articulaciones_bloqueadas_id.reserve(articulaciones_bloqueadas.size());
    for (const auto& str : articulaciones_bloqueadas)
      articulaciones_bloqueadas_id.emplace_back(model_complete.getJointId(str));

    // SI SE QUIERE QUE EL BRAZO INMOBILIZADO TENGA OTRA CONFIGURACION
    // TIENE QUE HACERSE AQUI!!
    auto q = pinocchio::neutral(model_complete);

    // Se crea el modelo reducido
    pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_id, q, model);
    
    // Se inicializa el estado del robot
    data = pinocchio::Data(model);

    // Calcula la Jacobiana completa
    q = pinocchio::neutral(model);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // Calcula la Jacobiana con respecto al extremo del brazo
    Eigen::MatrixXd J(6, model.nv); J.setZero();
    pinocchio::FrameIndex frame_id = model.getFrameId(end_effector_link);
    std::cout << "frame_id: " << frame_id << "\n";
    std::cout << "n_frames: " << model.frames.size() << "\n";

    pinocchio::getFrameJacobian(model, data, frame_id,
              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    Eigen::MatrixXd Jb = J.block<6, 6>(0,0);
    Eigen::MatrixXd Jm = J.block(0, 6, 6, 7);

    // Calcula las derivadas de la jacobiana
    auto v = Eigen::VectorXd::Random(model.nv);
    Eigen::MatrixXd dJ(6, model.nv); dJ.setZero();
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
    pinocchio::getFrameJacobianTimeVariation(model, data, frame_id,
              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
    auto dJb = dJ.block<6, 6>(0, 0);
    auto dJm = dJ.block(0, 6, 6, 7);

    // Calcula la matriz de inercia
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    // Se guardan las submatrices de inercia
    auto Hb  = data.M.block<6, 6>(0, 0);
    auto Hbm = data.M.block(0, 6, 6, 7);
    auto Hm  = data.M.block(6, 6, 7, 7);
    auto Hm_ = Hm - (Hbm.transpose() * Hb.inverse() * Hbm);

    std::cout << "Complete Mii:\n" << Hm_ << '\n';

    // Calcula los jacobianos pedidos
    Eigen::MatrixXd Jg  = Jm - Jb * (Hb.inverse() * Hbm);
    Eigen::MatrixXd dJg = dJm - dJb * (Hb.inverse() * Hbm);
    
    // Imprime todas las jacobianas
    std::cout << "Complete J:\n" << J << '\n';
    std::cout << "Jb:\n" << Jb << '\n';
    std::cout << "Jm:\n" << Jm << '\n';
    std::cout << "Complete dJ:\n" << dJ << '\n';
    std::cout << "dJb:\n" << dJb << '\n';
    std::cout << "dJm:\n" << dJm << '\n';
    std::cout << "Jg:\n" << Jg << '\n';
    std::cout << "dJg:\n" << dJg << '\n';
    std::cout << "doq_q:\n" << dJg*dot_q << '\n';
  }
}; //namespace