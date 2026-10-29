#pragma once
#include "data_types.hpp"
#include <memory>

namespace rm
{
    class TrackerBase
    {
    protected:
        int id_;
        TargetType type_;
        int missed_count_ = 0;    
    public:

        TrackerBase(int id,TargetType type) : id_(id),type_(type) {}
        virtual ~TrackerBase()=default;

        virtual void update(const Observation& obs,double dt)=0;
        virtual void predict(double dt) = 0;
        virtual TargetSnapshot getSnapshot() const = 0;
        
        int getId() const {return id_; }
        TargetType getType() const {return type_;}
        
        void markMiss() {missed_count_++;}
        void clearMiss() {missed_count_ =0;}
        int getMissCount() const {return missed_count_;}
    };
}