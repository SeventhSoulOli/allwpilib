#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <span>
#include "glass/other/Field2D.h"
#include "DataLog.h"

namespace sapphire{


// Field2D model from logs
class LogField2DModel : public glass::Field2DModel {

 public:

  static constexpr const char* kType = "Field2d";

  class ObjectModel : public glass::FieldObjectModel{
   public:
    ObjectModel(std::string_view name, EntryData * entry)
        : m_name{name}, m_entry{entry} {}
    ~ObjectModel() override = default;
    const char* GetName() const override { return m_name.c_str(); }
    EntryData * GetEntry() const { return m_entry; }

    void UpdatePoses(const wpi::log::DataLogRecord& value);

    void Update() override {}
    void Update(double time) {
        auto value = m_entry->GetRecordAt((time)*1e6);
        if (value.GetEntry() != -1) {
            UpdatePoses(value);
        }
    }
    bool Exists() override { return true; }
    bool IsReadOnly() override { return false; }

    std::span<const frc::Pose2d> GetPoses() override { return m_poses; }
    void SetPoses(std::span<const frc::Pose2d> poses) override;
    void SetPose(size_t i, frc::Pose2d pose) override;
    void SetPosition(size_t i, frc::Translation2d pos) override;
    void SetRotation(size_t i, frc::Rotation2d rot) override;

   private:

    std::string m_name;
    EntryData *m_entry;

    std::vector<frc::Pose2d> m_poses;
  };
  // path is to the table containing ".type", excluding the trailing /
  LogField2DModel(DataLogModel& model, std::string_view path, float& nowRef);
  ~LogField2DModel() override;

  const char* GetPath() const { return m_path.c_str(); }
  const char* GetName() const { return m_nameValue.c_str(); }

  void Update() override;
  bool Exists() override {return true;}
  bool IsReadOnly() override {return true;}

  glass::FieldObjectModel* AddFieldObject(std::string_view name) override;
  void RemoveFieldObject(std::string_view name) override;
  void ForEachFieldObject(
      wpi::function_ref<void(glass::FieldObjectModel& model, std::string_view name)>
          func) override;

 private:
  DataLogModel& m_model;
  std::string m_path;
  EntryNode* m_node;
  std::string m_nameValue;
  float& m_nowRef;
  float m_now;
  using Objects = std::vector<std::unique_ptr<ObjectModel>>;
  Objects m_objects;

  std::pair<Objects::iterator, bool> Find(std::string_view fullName);
};


} // namespace sapphire
